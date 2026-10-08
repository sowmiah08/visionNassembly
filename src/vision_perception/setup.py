import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vision_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Object, gripper and assembly description files.
        *[(os.path.join('share', package_name, 'config', sub), glob(f'config/{sub}/*.yaml'))
          for sub in ('objects', 'grippers', 'assemblies')],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zozo',
    maintainer_email='sowmiah.jerom@gmail.com',
    description='Perception, assembly and evaluation for the SO-101 dual-arm workcell',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gt_full_pick_insert = vision_perception.gt_full_pick_insert:main',
        ],
    },
)
