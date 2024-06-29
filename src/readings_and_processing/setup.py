from setuptools import find_packages, setup

package_name = 'readings_and_processing'

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
    maintainer='apptronik',
    maintainer_email='abdulwakil.ola@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_to_ackerman_node = readings_and_processing.teleop_twist_to_ackerman:main',
            'make_circle_node = readings_and_processing.make_circle:main'
        ],
    },
)
