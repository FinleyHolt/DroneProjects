from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flyby_depth'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Finley Holt',
    maintainer_email='finley@example.com',
    description='Monocular depth estimation with rangefinder scale correction for Flyby F-11 UAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_estimation_node = flyby_depth.depth_estimation_node:main',
        ],
    },
)
