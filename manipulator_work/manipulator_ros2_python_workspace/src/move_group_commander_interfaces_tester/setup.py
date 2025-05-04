from setuptools import find_packages, setup

package_name = 'move_group_commander_interfaces_tester'

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
    maintainer='mscrobotics2425laptop27',
    maintainer_email='winterbottomelliot@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_group_commander_interfaces_tester_node = move_group_commander_interfaces_tester.move_group_commander_interfaces_tester_node:main'
        ],
    },
)
