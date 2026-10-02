from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'so101_description'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

    # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

    # launch files
        (
            'share/' + package_name + '/launch',
            glob('launch/*.py')
        ),

    # world / sdf files
        (
            'share/' + package_name + '/worlds',
            glob('worlds/*.sdf')
        ),

    # URDF / Xacro / fragments
        (
            'share/' + package_name + '/urdf',
            glob('urdf/*.urdf') +
            glob('urdf/*.xacro') +
            glob('urdf/*.fragment') +
            glob('urdf/*.xml')
        ),

    # meshes / assets
        (
            'share/' + package_name + '/assets',
            glob('assets/**/*.stl', recursive=True)
        ),

    # procedurally generated workcell / object meshes
        (
            'share/' + package_name + '/meshes',
            glob('meshes/*.stl')
        ),

    #config
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        )
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='zozo',
    maintainer_email='sowmiah.jerom@gmail.com',

    description='Robot description package for SO101 setups',
    license='Apache-2.0',

    extras_require={
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [],
    },
)