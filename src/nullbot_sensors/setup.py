from setuptools import find_packages, setup

package_name = 'nullbot_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tt18',
    maintainer_email='tt18@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ultrasonic_node = nullbot_sensors.ultrasonic_node:main',
            'range_kalman_filter = nullbot_sensors.range_kalman_filter:main',
            'vl53l0x_triplet_node = nullbot_sensors.vl53l0x_triplet_node:main',
            'dataset_recorder_3range_ackermann = nullbot_sensors.dataset_recorder_3range_ackermann:main',


        ],
    },
)
