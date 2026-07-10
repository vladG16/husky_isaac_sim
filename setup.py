from setuptools import find_packages, setup

package_name = 'husky_sim_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vladG16',
    maintainer_email='you@example.com',
    description='Constant /cmd_vel publisher for driving the Husky in Isaac Sim (ROS 2 Jazzy).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'constant_cmd = husky_sim_test.constant_cmd:main',
        ],
    },
)
