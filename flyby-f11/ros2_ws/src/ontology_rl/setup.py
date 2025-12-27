from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ontology_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'gymnasium>=0.29.0',
        'numpy>=1.24.0',
        'stable-baselines3>=2.0.0',
    ],
    zip_safe=True,
    maintainer='Finley Holt',
    maintainer_email='finley@example.com',
    description='Gymnasium-based RL training environments for F-11 ontology-constrained autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_env_node = ontology_rl.nodes.gym_env_node:main',
        ],
    },
)
