from setuptools import find_packages, setup

package_name = 'sonar_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide Colabella',
    maintainer_email='davidecola16@gmail.com',
    description='Ping1D sonar driver node for PoliTOcean Nereo ROV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node     = sonar_pkg.sonar_node:main',
            'sonar_sim_node = sonar_pkg.sonar_sim_node:main',
        ],
    },
)
