from setuptools import find_packages, setup

package_name = 'husky_sim_nav'

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
    maintainer='lab',
    maintainer_email='vgunichev2022@fau.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_nav = husky_sim_nav.waypoint_navigator_pd_sim_standalone:main',
            'waypoint_nav_viz = husky_sim_nav.waypoint_navigator_pd_sim_viz:main',
        ],
    },
)
