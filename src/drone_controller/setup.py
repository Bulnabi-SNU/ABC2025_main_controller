from setuptools import setup
import os
from glob import glob

package_name = 'drone_controller'

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
            'pwm_test = drone_controller.servo_pwm_test:main',
            'control01 = drone_controller.controller_01:main',
            'test01 = drone_controller.test_01:main',
            'test02 = drone_controller.test_02_approach:main',
            'obstacle_speaker = drone_controller.obstacle_speaker:main'
        ],
    },
)
