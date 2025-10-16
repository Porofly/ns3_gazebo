from setuptools import find_packages, setup

package_name = 'forward_follower_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Forward follower robot that maintains constant x velocity and follows leader y velocity',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_follower_node = forward_follower_robot.forward_follower_node:main',
        ],
    },
)
