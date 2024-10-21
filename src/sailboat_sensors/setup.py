from setuptools import setup, find_packages

package_name = 'sailboat_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alber',
    maintainer_email='albert.yang.sun@gmail.com',
    description='AirMar and Anemometer sensors onboard sailboat',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'anemometer = sailboat_sensors.anemometer.anemometer_node:main',
                'airmar = sailboat_sensors.airmar.airmar_node:main',
        ],
    },
)
