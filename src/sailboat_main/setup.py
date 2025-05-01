from setuptools import setup, find_packages


package_name = 'sailboat_main'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo = sailboat_main.servo.servo_node:main',
            'radio = sailboat_main.radio.radio_node:main',
            # 'main_algo = sailboat_main.main_algo.main_algo:main',
            'main_algo = sailboat_main.main_algo.main_algo:main',
            'sail_algorithm = sailboat_main.main_algo.sail_algorithm:main',
            'trim_sail = sailboat_main.trim_sail.trim_sail:main',
            'teensy = sailboat_main.teensy.teensy_node:main',
            'mux = sailboat_main.mux.mux_node:main',
            'sail_algo = sailboat_main.main_algo.sail_algo:main',
            'algo = sailboat_main.main_algo.algo:main',
        ],
    },
)
