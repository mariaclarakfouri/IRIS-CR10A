from setuptools import setup

package_name = 'dobot_coletas'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'NOTICE.txt', 'LICENSE.cr10twin']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maria Clara Kfouri',
    maintainer_email='maria.kfourii@gmail.com',
    description='Coletas com o robô no sensor (palpação), com o explorador do cr10twin.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tactile_explorer = dobot_coletas.tactile_explorer:main',
            'palpation_logger = dobot_coletas.palpation_logger:main',
            'palpation_report = dobot_coletas.palpation_report:main',
            'ft_receiver = dobot_coletas.ft_receiver_node:main',
            'touch_receiver = dobot_coletas.touch_receiver_node:main',
            'force_sync = dobot_coletas.force_sync_node:main',
            'sim_force_bridge = dobot_coletas.sim_force_bridge:main',
        ],
    },
)
