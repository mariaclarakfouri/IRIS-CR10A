from glob import glob

from setuptools import find_packages, setup

package_name = 'dobot_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'NOTICE.txt']),
        ('share/' + package_name + '/robot3d', ['robot3d/cr10.urdf', 'robot3d/NOTICE.txt']),
        ('share/' + package_name + '/robot3d/meshes/cr10', glob('robot3d/meshes/cr10/*.STL')),
        ('share/' + package_name + '/icons',
            ['icons/up_arrow.png', 'icons/down_arrow.png', 'icons/check.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maria',
    maintainer_email='clara.trazzi@gmail.com',
    description='Interface gráfica (PyQt5) para controle do braço Dobot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dobot_gui = dobot_gui.dobot_gui_node:main',
        ],
    },
)
