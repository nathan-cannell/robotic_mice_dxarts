import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mouse_global'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/mouse_utils.py']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mouse',
    maintainer_email='ewidger@uw.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'presence_mux = mouse_global.presence_mux:main',
            'fuel_monitor = mouse_global.fuel_mux:main',
            'id_service = mouse_global.id_srv:main',
        ],
    },
)
