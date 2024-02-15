import os
from glob import glob
from setuptools import setup,find_packages

package_name = 'basic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*.launch.py')),
            (os.path.join('share', package_name,"config"), glob('config/*')),
            (os.path.join('share', package_name,"rviz"), glob('rviz/*')),
            (os.path.join('share', package_name,"map"), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ken20020209',
    maintainer_email='ken20020209@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'RobotDogConnector = basic.RobotDogConnector:main',
        'Camera = basic.Camera:main',
        'Navigation = basic.Navigation:main',
        ],
    },
)
