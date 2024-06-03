from setuptools import setup

package_name = 'sailboat_sensors'

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
    maintainer='alber',
    maintainer_email='albert.yang.sun@gmail.com',
    description='AirMar and Anemometer sensors onboard sailboat',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'read_wind = sailboat_sensors.read_wind:main',
                'test_read_wind = sailboat_sensors.test_read_wind:main',
                'read_airmar = sailboat_sensors.read_airmar:main',
                'listener = sailboat_sensors.test_listener:main',
        ],
    },
)
