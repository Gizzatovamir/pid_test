from setuptools import find_packages, setup
import os
import glob

package_name = 'simple_pid_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('simple_pid_robot/*.py')),
        (os.path.join('share', package_name, "config"), glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, "launch"), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amir',
    maintainer_email='gizzatovamir777@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'field = simple_pid_robot.field:main',
            'robot = simple_pid_robot.robot:main',
        ],
    },
)
