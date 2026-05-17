import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gui_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/images', glob('gui_pkg/images/*')),
        ('share/' + package_name + '/qml', glob('gui_pkg/qml/*.qml')),
        ('share/' + package_name + '/qml/components', glob('gui_pkg/qml/components/*.qml')),
        ('share/' + package_name + '/qml/assets/icons', glob('gui_pkg/qml/assets/icons/*.svg')),
        ('share/' + package_name + '/qml/assets/logo', glob('gui_pkg/qml/assets/logo/*.svg')),
    ],
    install_requires=[
        'setuptools',
        'std_msgs', 
        'sensor_msgs',
        'diagnostic_msgs'
    ],
    zip_safe=True,
    maintainer='Davide Colabella',
    maintainer_email='davidecola16@gmail.com',
    description='GUI for PoliTOcean Nereo ROV using QML',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node     = gui_pkg.gui_node:main',
            'rov_sim_node = gui_pkg.rov_sim_node:main',
        ],
    },
)