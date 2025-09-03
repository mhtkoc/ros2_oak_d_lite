from setuptools import setup
import os
from glob import glob

package_name = 'oak_d_lite'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files located under launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iftach',
    maintainer_email='iftahnaf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo = oak_d_lite.stereo:main',
            'stereo_imu = oak_d_lite.stereo_imu:main',
            'depth_map = oak_d_lite.depth_map:main',
            'pointcloud = oak_d_lite.pointcloud:main'
        ],
    },
)
