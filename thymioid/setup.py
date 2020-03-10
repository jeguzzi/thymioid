from setuptools import setup
# from setuptools import find_packages
from glob import glob

package_name = 'thymioid'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    keywords=['Mighty Thymio'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A ROS driver for the Mighty Thymio robot',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_menu = thymioid.wifi_node_pipe:main',
            'ups = thymioid.ups_node:main',
            'ups_ui = thymioid.ups_ward_node:main',
            'ui = thymioid.ui_node:main',
            'camera_pitch_controller = thymioid.camera_pitch_controller:main',
            'demo_menu = thymioid.demo_menu:main'
        ],
    },
)
