from setuptools import find_packages, setup

package_name = 'driver_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'movimenti_interfacce'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'driver_server = driver_server.driver_server:main',
                'asincronia = driver_server.asincronia:main',
                'mover = driver_server.mover:main',
                'provaAlessandro = driver_server.provaAlessandro:main',
		'provaAndrea = driver_server.provaAndrea:main',
        ],
    },
)
