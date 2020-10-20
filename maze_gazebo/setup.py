import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'maze_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['launch/**', 'model/**', 'worlds/**']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, "worlds"), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'model', 'challenge_maze'), glob("model/challenge_maze/**.*")),
        (os.path.join('share', package_name, 'model', 'marker0'), glob("model/marker0/**.*")),
        (os.path.join('share', package_name, 'model', 'marker0', 'materials'), glob("model/marker0/materials/**.*")),
        (os.path.join('share', package_name, 'model', 'marker0', 'materials', 'textures'), glob("model/marker0/materials/textures/**.*")),
        (os.path.join('share', package_name, 'model', 'marker0', 'meshes'), glob("model/marker0/meshes/**.*")),
        (os.path.join('share', package_name, 'model', 'marker26_8cm'), glob("model/marker26_8cm/**.*")),
        (os.path.join('share', package_name, 'model', 'marker26_8cm', 'material'), glob("model/marker26_8cm/material/**.*")),
        (os.path.join('share', package_name, 'model', 'marker26_8cm', 'material', 'scripts'),
         glob("model/marker26_8cm/material/scripts/**.*")),
        (os.path.join('share', package_name, 'model', 'marker26_8cm', 'material', 'textures'),
         glob("model/marker26_8cm/material/textures/**.*")),
        (os.path.join('share', package_name, 'model', 'turtlebot3_waffle'), glob("model/turtlebot3_waffle/**.*")),
        (os.path.join('share', package_name, 'model', 'turtlebot3_waffle', 'meshes'),
         glob("model/turtlebot3_waffle/meshes/**.*"))
    ],
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heimv',
    maintainer_email='victor.heim@epitech.eu',
    description='challenge world',
    license='Apache-2.0',
    tests_require=['pytest']
)
