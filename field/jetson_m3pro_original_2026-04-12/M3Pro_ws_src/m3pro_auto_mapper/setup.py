from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'm3pro_auto_mapper'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@yahboom.com',
    description='Enhanced auto exploration mapping for Yahboom M3Pro.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_explore_mapper   = m3pro_auto_mapper.auto_explore_mapper:main',
            'auto_explore_finisher = m3pro_auto_mapper.auto_explore_finisher:main'
        ],
    },
)
