import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'maze_gazebo'

def map_data_files(src_dir):
    files_map = [
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ]
    for walk_dir in src_dir:
        for root, dirs, files in os.walk(walk_dir):
            current_dir = os.path.join('share', package_name, root)
            current_files = []
            for file in files:
                current_files.append(os.path.join(root, file))
            files_map.append((current_dir, current_files))
    return files_map

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['launch/**', 'model/**', 'worlds/**']),
    data_files=map_data_files(["launch", "worlds", "model"]),
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heimv',
    maintainer_email='victor.heim@epitech.eu',
    description='challenge world',
    license='Apache-2.0',
    tests_require=['pytest']
)
