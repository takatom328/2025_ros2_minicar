from setuptools import find_packages, setup

package_name = 'nullbot_actuators'

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
            #'servo_node = nullbot_actuators.servo_node:main',
            #'servo_gpio_node = nullbot_actuators.servo_gpio_node:main',
            'pca9685_steer_node = nullbot_actuators.pca9685_steer_node:main',
    ],
},

)
