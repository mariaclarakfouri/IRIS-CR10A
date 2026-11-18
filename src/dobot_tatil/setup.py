from setuptools import setup

package_name = 'dobot_tatil'

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
    description='Leitura dos sensores táteis 4x4 e 5x5 (placa STM32 por USB).',
    license='Apache-2.0',
    tests_require=['pytest'],
)
