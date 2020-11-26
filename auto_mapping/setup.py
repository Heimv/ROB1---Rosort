import os
from setuptools import setup

package_name = 'auto_mapping'

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
    packages=[package_name],
    data_files=map_data_files([]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heimv',
    maintainer_email='victor.heim@epitech.eu',
    description='automatically map the environment of the robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_map = auto_mapping.auto_map:main'
        ],
    },
)
