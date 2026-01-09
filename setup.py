from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cli_spawn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # World, launch and robot file
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='venus',
    maintainer_email='ttiemdang@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cli_spawn = cli_spawn.cli_spawn:main',
            'control_position = cli_spawn.control_pos:main',
            'read_end_point = cli_spawn.read_end_point:main',
            'ga_calc = cli_spawn.calc:main',
            'test_ctrl = cli_spawn.test_control:main',
            'test_reset = cli_spawn.test_reset:main',
            'test_getpos = cli_spawn.test_getpos:main',
            'test_getworldpose = cli_spawn.test_get_worldpose:main',
            'test_spawn = cli_spawn.test_spawn:main'
        ],
    },
)
