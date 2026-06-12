import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'kill_switch'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seungjae99',
    maintainer_email='seungjaechoi9@gmail.com',
    description='Kill Switch node for autonomous tugboat safety monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kill_switch_node = kill_switch.kill_switch_node:main',
        ],
    },
)
