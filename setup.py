from glob import glob
import os
from setuptools import setup

package_name = 'hwr_indoor_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={'hwr_indoor_navigation': ['capture-lidar-data-and-print-to-stdout']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('model/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='PiBot@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_lidar_data = hwr_indoor_navigation.publisher_lidar_data:main',
        ],
    },
)
