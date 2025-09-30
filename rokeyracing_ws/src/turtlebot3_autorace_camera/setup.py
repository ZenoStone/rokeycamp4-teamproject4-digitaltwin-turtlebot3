from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_autorace_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/calibration/extrinsic_calibration',
            glob('calibration/extrinsic_calibration/*.yaml')),
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
            'image_compensation = turtlebot3_autorace_camera.image_compensation:main',
            'camera_image_pub = turtlebot3_autorace_camera.camera_image_pub:main',
            'image_processing = turtlebot3_autorace_camera.image_processing:main',            
            'image_projection = turtlebot3_autorace_camera.image_projection:main'
        ],
    },
)
