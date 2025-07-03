from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('detection/msg/*.msg')),
        (os.path.join('share', package_name), glob('srv/*.srv')),
        (os.path.join('share', package_name), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group8',
    maintainer_email='shamoung@kth.se',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = detection.detection:main',
            'fluffy_detection = detection.fluffy_detection:main',
            'dl_detection = detection.dl_detection_cls:main'
        ],
    },
    
)
