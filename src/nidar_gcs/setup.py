from setuptools import setup
import os
from glob import glob

package_name = 'nidar_gcs'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('nidar_gcs/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NIDAR Autonomy Team',
    maintainer_email='team@nidar.local',
    description='NIDAR AirMouse Ground Control Station (GCS) Web Dashboard',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gcs_server = nidar_gcs.server:main',
        ],
    },
)
