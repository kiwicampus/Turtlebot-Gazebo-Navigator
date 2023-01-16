from setuptools import setup

package_name = 'low_level_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas Rocha Pacheco',
    maintainer_email='nicolas.rocha@kiwibot.com',
    description='This package has the nodes that are required to carry out the first section of the 22Q1 hiring process for Kiwibot.',
    license='Apache 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller_interface_test = low_level_pkg.util.controller_interface:test_controller_server",
            "planner_interface_test = low_level_pkg.util.planner_interface:test_planner_server",
            "behavior_interface_test = low_level_pkg.util.behavior_interface:test_behavior_server",
            "behavior_tree = low_level_pkg.behavior_tree:behavior_tree",
        ],
    },
)