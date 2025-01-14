from setuptools import setup
import os
from glob import glob

package_name = 'shit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaewon',
    maintainer_email='chaewon@snu.ac.kr',
    description='Skyborne Hazard Interception Tool controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control01 = shit_controller.controller_01:main',
        ],
    },
)
