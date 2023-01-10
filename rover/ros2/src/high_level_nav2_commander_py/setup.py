from setuptools import setup

package_name = 'high_level_nav2_commander_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ada',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'high_level_commander_node = high_level_nav2_commander_py.high_level_commander_node:main'
        ],
    },
)
