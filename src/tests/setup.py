from setuptools import find_packages, setup

package_name = 'tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),  # Include launch files
        ('share/' + package_name + '/config', ['config/ekf.yaml']),  # Include config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_subscriber_node = tests.gps_subscriber_node:main',
            'path_perimeter_node = tests.path_perimeter_node:main',
            'path_planning_node = tests.path_planning_node:main',
            'robot_teleop_node = tests.robot_teleop_node:main',
            'path_generator_node = tests.path_generator_node:main',
            'navigation_node = tests.navigation_node:main',
            'pose_calculator_node = tests.pose_calculator_node:main',
            'IMU_node = tests.IMU_node:main',
            'stanly_navigation_node = tests.stanly_navigation_node:main',
            'collision_avoidance_node = tests.collision_avoidance_node:main',
            'stanly_navigation_node_original = tests.stanly_navigation_node_original:main',
            'abd_collivoid_node = tests.abd_collivoid_node:main',
            'abd_stannav_node = tests.abd_stannav_node:main',

            
        ],
    },
)

