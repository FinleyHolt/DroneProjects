"""Setup file for vampire_bridge package."""

from setuptools import setup, find_packages

package_name = 'vampire_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Finley Holt',
    maintainer_email='finley@example.com',
    description='ROS 2 bridge to the Vampire theorem prover for UAV ontological reasoning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vampire_node = vampire_bridge.vampire_node:main',
        ],
    },
)
