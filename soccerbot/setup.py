import os
from glob import glob
from setuptools import setup

package_name = 'soccerbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doug de Jesus',
    maintainer_email='drd8913@nyu.edu',
    description='SoccerBot: Formation control for soccer players',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'formation_controller = soccerbot.formation_controller:main',
            'distance_sensor = soccerbot.distance_sensor:main',
            'collision_detection = soccerbot.collision_detection:main',
        ],
    },
)
