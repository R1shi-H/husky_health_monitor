from glob import glob

from setuptools import find_packages, setup

package_name = 'husky_health_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rishi Halker',
    maintainer_email='r2halker@gmail.com',
    description='Health monitoring dashboard for the Husky A300 and MORS Code software',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_monitor = husky_health_monitor.monitor_node:main',
            'health_status = husky_health_monitor.cli:main',
            'mock_husky = husky_health_monitor.mock_husky:main',
        ],
    },
)
