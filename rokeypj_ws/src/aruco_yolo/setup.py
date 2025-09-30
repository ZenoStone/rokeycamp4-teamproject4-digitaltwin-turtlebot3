from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/calibration_params.yaml','config/camera_params.yaml','config/kinematics.yaml']), 

        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey2',
    maintainer_email='rokey2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_yolo.aruco_detector:main',  
            'yolo_detector = aruco_yolo.yolo_detector:main',
            'camera_pub = aruco_yolo.camera_pub:main',
            'mission_controller = aruco_yolo.mission_controller:main',       
            'detect_level_crossing = aruco_yolo.detect_level_crossing:main',
            'detect_lane = aruco_yolo.detect_lane:main',
        ],
    },
    package_data={
        package_name: [
            'config/calibration_params.yaml',
        ],
    },
)
