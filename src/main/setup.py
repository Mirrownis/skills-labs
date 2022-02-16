from setuptools import setup

package_name = 'main'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eike Schultz',
    maintainer_email='eike-schultz@web.de',
    description='The main package to the skills labs ROS network',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui = main.node_ui_core:main',
            'log = main.node_log_core:main',
            'plan = main.node_plan_core:main',
        ],
    },
)
