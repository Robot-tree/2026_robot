from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rtreebot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rtree',
    maintainer_email='rtree@example.com',
    description='웹 UI와 로봇을 연결하는 브릿지 패키지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'delivery_bridge = rtreebot.delivery_bridge_node:main',
            'delivery_ctrl = rtreebot.delivery_ctrl:main',
        ],
    },
)
