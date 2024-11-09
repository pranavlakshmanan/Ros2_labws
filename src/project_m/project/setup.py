from setuptools import find_packages, setup
import os 
from glob import glob 
package_name = 'project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'),glob('launch/*')), 
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'textures'), glob('textures/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'torch',
    ],
    zip_safe=True,
    maintainer='pranavlakshmanan',
    maintainer_email='pranavlakshman@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "telop = project.teleop_control:main",
            'mecanum_controller = project.mecanum_controller:main',
            'ppo_train = project.ppo_train:main',# Added PPO training entry point
            'game = project.game_teleop:main',
            "telop_m = project.teleop_control_m:main" 
        ],
    },
)