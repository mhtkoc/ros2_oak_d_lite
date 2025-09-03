import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class OakPointCloud(Node):
    def __init__(self):
        super().__init__('oak_pointcloud')

        self.publisher = self.create_publisher(PointCloud2, '/oak/points', 10)

        # Pipeline - VPU optimizasyonu için
        self.pipeline = dai.Pipeline()
        monoL = self.pipeline.createMonoCamera()
        monoR = self.pipeline.createMonoCamera()
        stereo = self.pipeline.createStereoDepth()

        # Düşük çözünürlük = daha hızlı işlem
        monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # 400p daha hızlı
        monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # VPU optimizasyonu için stereo ayarları
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)  # Daha az noise
        stereo.setSubpixel(False)  # VPU'da daha hızlı
        stereo.setLeftRightCheck(True)  # Donanımda filtreleme
        stereo.setConfidenceThreshold(200)  # Düşük güvenilir noktaları filtrele
        
        # Depth filtering (VPU'da yapılır) - uyumlu metodlar
        stereo.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        
        # Temporal ve spatial filtering (eğer mevcut ise)
        try:
            stereo.setTemporalFilter(True, alpha=0.4, delta=20)
            stereo.setSpatialFilter(True, alpha=0.5, delta=20, numIterations=1)
            stereo.setSpeckleRemoval(True)
        except AttributeError:
            # Bu metodlar mevcut değilse, temel filtrelemeyi kullan
            pass
        
        # RGB align kaldır (daha hızlı)
        # stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        
        # Deprecated çağrıları kaldır
        # stereo.setOutputDepth(True)
        # stereo.setOutputRectified(True)

        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)

        xout_depth = self.pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        self.device = dai.Device(self.pipeline)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        in_depth = self.q_depth.tryGet()
        if in_depth is None:
            return

        depth_frame = in_depth.getFrame().astype(np.float32)
        h, w = depth_frame.shape
        
        # DepthAI genelde mm cinsinden depth verir, m'ye çevir
        depth_frame = depth_frame / 1000.0  # mm -> m
        
        # CPU'da downsampling (her N piksel bir al)
        downsample_factor = 2  # Her 2 piksel bir al = 4x daha az nokta
        depth_frame = depth_frame[::downsample_factor, ::downsample_factor]
        h, w = depth_frame.shape
        
        # Geçersiz depth değerlerini filtrele (0 ve çok uzak değerler)
        valid_mask = (depth_frame > 0.01) & (depth_frame < 3.0)  # 1cm - 2m arası

        # Basit XYZ dönüşümü (fx, fy, cx, cy kalibre edilmeli)
        fx = fy = 250.0  # Downsample nedeniyle focal length yarıya düşer
        cx, cy = w/2, h/2
        xs, ys = np.meshgrid(np.arange(w), np.arange(h))
        
        # Sadece geçerli depth değerleri için hesapla
        X = (xs - cx) * depth_frame / fx
        Y = (ys - cy) * depth_frame / fy
        Z = depth_frame
        
        # Geçerli noktaları filtrele
        points_all = np.stack((X, Y, Z), axis=-1)
        points = points_all[valid_mask]
        
        # Ekstra CPU filtreleme (isteğe bağlı)
        # Random subsampling - nokta sayısını daha da azalt
        if len(points) > 5000:  # 5000'den fazla nokta varsa
            indices = np.random.choice(len(points), 5000, replace=False)
            points = points[indices]
        
        # Debug: nokta sayısını yazdır
        #self.get_logger().info(f'Valid points: {len(points)}/{h*w}')

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "oakd_link"
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OakPointCloud()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
