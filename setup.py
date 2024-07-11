from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roomba_oi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'pyroombaadapter'],
    zip_safe=True,
    maintainer='Jackson Empey',
    maintainer_email='jacksonempey98@gmail.com',
    description='This package acts as a bridge between ros and a roomba',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roomba_oi_main = roomba_oi.roomba_oi_main:main',
            'roomba_oi_main_sim = roomba_oi.roomba_oi_main_sim:main',
            'roomba_teleop_adapter = roomba_oi.roomba_teleop_adapter:main'
        ],
    },
)
