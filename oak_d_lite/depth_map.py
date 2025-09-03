import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import PointCloud2
import depthai as dai
import time
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

class Bridge(Node):

    def __init__(self):
        super().__init__('oak_d_lite')

        self.rate = 20  # 20hz target loop
        self.create_pipeline()
        self.run()

    def create_pipeline(self):
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)

        self.xoutDepth.setStreamName('depth')

        # Properties
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)

        # Configure stereo preset compatible with installed depthai
        try:
            self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.LOW_DENSITY)
        except AttributeError:
            # Fallback for versions without LOW_DENSITY
            self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(False)
        # Align depth to rectified left (compat across versions)
        try:
            self.stereo.setDepthAlign(dai.node.StereoDepth.DepthAlign.RECTIFIED_LEFT)
        except AttributeError:
            self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

        # Linking
        self.monoLeft.out.link(self.stereo.left)
        self.monoRight.out.link(self.stereo.right)
        self.stereo.depth.link(self.xoutDepth.input)

        # ros publisher (point cloud only)
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 1)

        # Target output size for low-res depth (width, height)
        self.depth_target_size = (320, 240)
        # Point cloud downsample stride (1 = every pixel)
        self.pc_stride = 2
        # Scaled intrinsics for resized depth
        self.K_scaled = None

        self.get_logger().info("initiate Oak-D-Lite ROS Depth Node (PointCloud only) ...")

    def run(self):
        with dai.Device(self.pipeline) as device:

            self.get_logger().info(f"Device ready for use ...")

            # Output queue for depth
            qDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

            calib = device.readCalibration()

            while True:
                start = time.time()

                try:
                    inDepth = qDepth.tryGet()
                except Exception:
                    self.get_logger().warn("Failed to get depth from depth queue")
                    continue

                now = self.get_clock().now().to_msg()

                # Build and publish point cloud from low-resolution depth
                if inDepth is not None:
                    depth_frame = inDepth.getFrame()  # uint16 depth in mm
                    h0, w0 = depth_frame.shape[:2]
                    try:
                        resized = cv2.resize(depth_frame, self.depth_target_size, interpolation=cv2.INTER_NEAREST)
                    except Exception:
                        resized = depth_frame  # fallback if resize fails

                    # Compute scaled intrinsics once based on actual sizes
                    if self.K_scaled is None:
                        try:
                            K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, w0, h0), dtype=np.float32)
                            sx = float(resized.shape[1]) / float(w0)
                            sy = float(resized.shape[0]) / float(h0)
                            fx = K[0, 0] * sx
                            fy = K[1, 1] * sy
                            cx = K[0, 2] * sx
                            cy = K[1, 2] * sy
                            self.K_scaled = (fx, fy, cx, cy)
                            self.get_logger().info(f"Intrinsics scaled fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
                        except Exception as e:
                            self.get_logger().warn(f"Failed to read calibration intrinsics: {e}")
                            # Fallback: approximate pinhole values if calibration not available
                            fx = fy = 430.0 * (float(resized.shape[1]) / 640.0)
                            cx = (resized.shape[1] - 1) / 2.0
                            cy = (resized.shape[0] - 1) / 2.0
                            self.K_scaled = (fx, fy, cx, cy)

                    # Build and publish point cloud
                    try:
                        fx, fy, cx, cy = self.K_scaled
                        depth_m = resized.astype(np.float32) / 1000.0
                        h, w = depth_m.shape
                        s = max(1, int(self.pc_stride))
                        us = np.arange(0, w, s, dtype=np.float32)
                        vs = np.arange(0, h, s, dtype=np.float32)
                        uu, vv = np.meshgrid(us, vs)
                        Z = depth_m[::s, ::s]
                        valid = Z > 0
                        if np.any(valid):
                            X = (uu - cx) / fx * Z
                            Y = (vv - cy) / fy * Z
                            pts = np.stack((X[valid], Y[valid], Z[valid]), axis=-1).astype(np.float32)

                            header = Header()
                            header.stamp = now
                            header.frame_id = 'oakd_link'
                            cloud = pc2.create_cloud_xyz32(header, pts.reshape(-1, 3))
                            self.pc_pub.publish(cloud)
                    except Exception as e:
                        self.get_logger().warn(f"PointCloud build failed: {e}")

                dt = time.time() - start

                if dt > (1 / self.rate) * 1.1:
                    self.get_logger().warn(f"Loop took too much time = {dt}")

                self.get_logger().info("Working ...", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)