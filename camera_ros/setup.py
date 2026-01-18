from setuptools import find_packages, setup
import glob, os

package_name = 'camera_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='12941516',
    maintainer_email='tesla0131@ajou.ac.kr',
    description='Package for calibration & publisher imgs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration = camera_ros.calibration:main',
            'publisher = camera_ros.publisher:main',
            'video_test = camera_ros.video_test:main',
        ],
    },
)
