from glob import glob
import os

from setuptools import find_packages
from setuptools import setup


package_name = 'opennav_cleaning_coverage'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lin Zheng',
    maintainer_email='zlin6364@hotmail.com',
    description=(
        'Indoor edge and complete-coverage planning for ROS 2 and Nav2.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'cleaning_coverage_server = '
                'opennav_cleaning_coverage.server:main'
            ),
        ],
    },
)
