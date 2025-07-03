from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_controller = controller.open_loop_controller:main',
            'cartesian_controller = controller.cartesian_controller:main',
            'joystick = controller.joystick:main',
            'object_following_controller = controller.object_following_controller:main',
            'detection_test = controller.detection_test:main',
            'arm_controller = controller.arm_controller:main',
            'path_planner = controller.path_planner:main'
        ],
    },
)
