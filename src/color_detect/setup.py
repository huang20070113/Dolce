from setuptools import find_packages, setup

package_name = 'color_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwj',
    maintainer_email='hwj@example.com',
    description='Color detection package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = color_detect.camera_node:main',
            'color_detector_node = color_detect.color_detector_node:main',
        ],
    },
)
