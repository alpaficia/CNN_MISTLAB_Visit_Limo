import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'limo_optitrack'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name), glob('limo_optitrack/scripts/*.py')),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dong Wang',
    maintainer_email='dong.wang950305@gmail.com',
    description='ROS 2 package for Limo with OptiTrack integration',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'limo_pos_publisher = limo_optitrack.scripts.limo_pos_publisher:main',
        ],
    },
)

