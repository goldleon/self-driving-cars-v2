from setuptools import setup

package_name = 'self_driving_car_pkg'

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
    maintainer='Anass Bouchtaoui',
    maintainer_email='bouchtaoui.anass@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = self_driving_car_pkg.video_recorder:main',
            'driver_node = self_driving_car_pkg.driver_node:main',
            'spawn_node = self_driving_car_pkg.self_driving_spawner:main'
        ],
    },
)
