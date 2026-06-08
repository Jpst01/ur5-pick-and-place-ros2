from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'ur5_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
          glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
          glob('worlds/*.sdf')),
        (os.path.join('share',package_name,'urdf'),
          glob('urdf/*.xacro')),
        (os.path.join('share',package_name,'config'),
          glob('config/*.yaml')),
        (os.path.join('share',package_name,'srdf'),
          glob('srdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpst',
    maintainer_email='jpst@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_pose = ur5_controller.move_to_pose:main',
            'pick_and_place = ur5_controller.pick_and_place:main',
            'box_pose_bridge = ur5_controller.box_pose_bridge:main',
            'detect_box = ur5_controller.detect_box:main',
            'pixel_to_world = ur5_controller.pixel_to_world:main',
        ],
    },
)
