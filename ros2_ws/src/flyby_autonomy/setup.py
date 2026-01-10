from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flyby_autonomy'

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
    description='ROS 2 autonomy node for Flyby F-11 UAV with ontology-driven behavior control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ontology_controller_node = flyby_autonomy.ontology_controller_node:main',
            'state_estimator_node = flyby_autonomy.state_estimator_node:main',
        ],
    },
)
