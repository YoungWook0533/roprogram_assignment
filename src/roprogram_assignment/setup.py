import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roprogram_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimyeonguk',
    maintainer_email='kimyeonguk@todo.todo',
    description='Package containing Topic+Service+Action+parameter+launch+custom interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node1 = roprogram_assignment.node1:main',
            'node2 = roprogram_assignment.node2:main',
            
        ],
    },
)
