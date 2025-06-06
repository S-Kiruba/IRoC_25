import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_drone_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages', ['resource/my_drone_description']),
    	('share/my_drone_description', ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/meshes', glob('meshes/*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiruba',
    maintainer_email='kiruba@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'my_joint_state_publisher = my_drone_description.joint_state_publisher:main', 
        ],
    },
)
