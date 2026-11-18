import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dobot_twin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'NOTICE.txt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'meshes', 'cr10'), glob('meshes/cr10/*.STL')),
        (os.path.join('share', package_name, 'meshes', 'celula'), glob('meshes/celula/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maria',
    maintainer_email='clara.trazzi@gmail.com',
    description='Gêmeo virtual do CR10 no Gazebo (espelho do robô real e modo simulação)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gemeo_espelho = dobot_twin.espelho_node:main',
        ],
    },
)
