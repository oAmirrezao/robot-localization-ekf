from setuptools import setup
import os
from glob import glob

package_name = 'robot_local_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@sut.ac.ir',
    description='Robot localization using EKF',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prediction_node = robot_local_localization.prediction_node:main',
            'measurement_node = robot_local_localization.measurement_node:main',
            'ekf_node = robot_local_localization.ekf_node:main',
            'test_node = robot_local_localization.test_node:main',
        ],
    },
)
