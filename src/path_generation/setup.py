from setuptools import setup

package_name = 'path_generation'

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
    maintainer='yoonho',
    maintainer_email='yoonho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dstar_bezier_path_planner = path_generation.dstar_bezier_path_planner:main',
            'astar_fixed_start = path_generation.astar_fixed_start:main',
            'astar_zed_start = path_generation.astar_zed_start:main',
            'astar_fixed_start_simulation = path_generation.astar_fixed_start_simulation:main',
            'Drone_Commander = path_generation.Drone_Commander:main',
            'publish_pcd = path_generation.publish_pcd:main'
            ],
    },
)
