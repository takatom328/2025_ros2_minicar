from setuptools import find_packages, setup

package_name = 'nullbot_teleop'

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
        'joy_to_servo = nullbot_teleop.joy_to_servo:main',
        'joy_to_cmd   = nullbot_teleop.joy_to_cmd:main',
    ],
},
)
