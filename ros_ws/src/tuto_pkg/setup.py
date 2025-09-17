from setuptools import find_packages, setup

package_name = 'tuto_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = tuto_pkg.simple_pub: main',
            'simple_subscriber = tuto_pkg.simple_sub: main',
            'simple_param_node = tuto_pkg.simple_parameter: main',
            'simple_service_node = tuto_pkg.simple_server_service: main',
            'simple_service_client = tuto_pkg.simple_server_client: main',
            'simple_action_server = tuto_pkg.simple_action_server: main',
            'simple_action_client = tuto_pkg.simple_action_client: main',
            'simple_lifecycle_node = tuto_pkg.lifecycle_node: main'
        ],
    },
)
