from setuptools import setup

package_name = 'paa5160e1_driver_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/paa5160e1.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ar-Ray-code',
    maintainer_email='ray255ar@gmail.com',
    description='Bringup launch for PAA5160E1 driver',
    license='Apache-2.0',
    tests_require=['pytest'],
)

