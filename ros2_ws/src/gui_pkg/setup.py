from setuptools import find_packages, setup

package_name = 'gui_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Salvatore Lo Sardo',
    maintainer_email='losardosalvatorejr@gmail.com',
    description='GUI for PoliTOcean Nereo ROV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = gui_pkg.gui_node:main',
        ],
    },
)
