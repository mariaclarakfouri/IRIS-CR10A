from setuptools import setup

package_name = 'dobot_celula'

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
    description='Leitura da célula de carga FA7155 (6 eixos) na ponta do CR10A.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'celula_carga = dobot_celula.celula_carga_node:main',
        ],
    },
)
