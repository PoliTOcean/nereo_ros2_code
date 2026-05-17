from glob import glob
from setuptools import find_packages, setup

package_name = 'web_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('web_pkg/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide Colabella',
    maintainer_email='davidecola16@gmail.com',
    description='Web-based ROV controller with rosbridge WebSocket bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server_node = web_pkg.web_server_node:main',
            'safety_node     = web_pkg.safety_node:main',
        ],
    },
)
