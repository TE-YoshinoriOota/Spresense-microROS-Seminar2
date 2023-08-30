import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'tb_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'param'),  glob('param/*.yaml')),
        (os.path.join('share', package_name, 'map'),    glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yohta',
    maintainer_email='yohta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
