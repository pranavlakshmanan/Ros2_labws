from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranavlakshmanan',
    maintainer_email='pranavlakshman79@gmail.com',
    description='Mecanum wheel controller for ping pong robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_controller = mecanum_controller.mecanum_controller:main',
        ],
    },
)