from setuptools import setup

package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.utils'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'image_publisher = camera_package.image_publisher:main',
            'person_detector = camera_package.person_detector:main',
        ],
    },
)
