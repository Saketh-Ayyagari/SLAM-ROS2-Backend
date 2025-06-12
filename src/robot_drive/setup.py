from setuptools import find_packages, setup
# imports for launch files
import os
from glob import glob

package_name = 'robot_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'numpy', 'pygame'],
    zip_safe=True,
    maintainer='tarkin',
    maintainer_email='sakethsarma07@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_node = robot_drive.joy_node:main',
            'pwm = robot_drive.pwm_convert:main',
            'send_power = robot_drive.send_power:main',
        ],
    },

)
