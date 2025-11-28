from setuptools import setup
from glob import glob
import os

package_name = 'outdoors_loc_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('outdoors_loc_nav/launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('outdoors_loc_nav/params/*.yaml')),
        (os.path.join('share', package_name, 'params'), glob('outdoors_loc_nav/params/*.lua')),
        (os.path.join('share', package_name, 'tf'), glob('outdoors_loc_nav/tf/*')),
        (os.path.join('share', package_name, 'assets', 'maps'), glob('outdoors_loc_nav/assets/maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergei Grichine',
    maintainer_email='slg@quakemap.com',
    description='Outdoor GPS SLAM Toolbox or Map Server bringup',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
