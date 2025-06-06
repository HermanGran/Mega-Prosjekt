from setuptools import find_packages, setup

package_name = 'cube_detection'

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
    maintainer='jon-h-vard',
    maintainer_email='jon.havard@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detector_node = cube_detection.cube_detector_node:main',
            'pose_estimator_node = cube_detection.pose_estimator_node:main',
            'pixel_size_detector = cube_detection.pixel_size_detector:main',
        ],
    },
)
