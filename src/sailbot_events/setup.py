from setuptools import find_packages, setup

package_name = 'sailbot_events'

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
    maintainer='root',
    maintainer_email='nikil.shyamsunder@gmail.com',
    description='Event Algorithms for Sailbot Competition',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'event_driver = sailbot_events.event_driver.event_driver_node:main',
        ],
    },
)
