import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'six_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch, map, rviz, config files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bouchier',
    maintainer_email='paul.bouchier@gmail.com',
    description='Run six can contest',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_republisher = six_can.odom_republisher:main',
            'capture_can = six_can.capture_can:main',
            'yaml_parser_node = six_can.yaml_parser_node:main',
            'can_chooser_node = six_can.can_chooser:main', 
            'six_can_runner = six_can.six_can_runner:main',
            'nav2pose = six_can.nav2pose:main',
            'can_chooser_test = test.can_chooser_test:main',
            'can_chooser_client = six_can.can_chooser_client:main',
            'basic_navigator_child = six_can.BasicNavigatorChild:main',
            'test_nav_and_move = test.test_nav_and_move:main',
            'jaws = six_can.jaws:main',
        ],
    },
)
