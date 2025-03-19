from setuptools import find_packages, setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/object_detection_launch.py']),  # ✅ 添加 launch 文件
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mscrobotics2425laptop7',
    maintainer_email='jiayi.zeng-2@postgrad.manchester.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'object_detection = object_detection.object_detection:main'
        
        ],
    },
)
