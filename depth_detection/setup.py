from setuptools import setup

package_name = 'depth_yolo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimgracy',
    maintainer_email='kimgracy@snu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_zed = depth_yolo_pkg.depth_zed:main',
            'depth_v4l2 = depth_yolo_pkg.depth_v4l2:main'
        ],
    },
)
