from setuptools import find_packages, setup

package_name = 'robot_car_ros'

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
    maintainer='nick-zh',
    maintainer_email='nichoruizhilkin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
  entry_points={
    'console_scripts': [
        'car_bridge_node = robot_car_ros.car_bridge_node:main',
    ],
},
)
