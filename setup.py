from setuptools import find_packages, setup

package_name = 'wpt_adjustment_turtlebot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/wpt_alignment.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='Hjin',
    maintainer_email='271123352+hjinyy@users.noreply.github.com',
    description='AprilTag based WPT coil alignment controller for TurtleBot using ROS2 cmd_vel.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['wpt_alignment_node = wpt_adjustment_turtlebot.wpt_alignment_node:main']},
)
