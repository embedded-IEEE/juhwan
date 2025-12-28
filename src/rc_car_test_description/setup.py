from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rc_car_test_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes', 'visual'), glob('meshes/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'collision'), glob('meshes/collision/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description=package_name,
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'twist_to_joint_cmd = rc_car_test_description.twist_to_joint_cmd:main',
            'scan_frame_rewriter = rc_car_test_description.scan_frame_rewriter:main',
            'odom_stamp_rewriter = rc_car_test_description.odom_stamp_rewriter:main',
        ],
    },
)
