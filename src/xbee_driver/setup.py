from setuptools import find_packages, setup

package_name = 'xbee_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'digi-xbee'],
    zip_safe=True,
    maintainer='lemezoth',
    maintainer_email='thomas.le_mezo@ensta-bretagne.org',
    description='Xbee driver for seasource buoy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbee = xbee_driver.xbee:main'
        ],
    },
)
