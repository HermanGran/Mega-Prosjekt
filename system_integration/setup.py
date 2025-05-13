from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'system_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/system_integration']),
        ('share/system_integration', ['package.xml']),
        # Installer launch-filer riktig
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tryvann',
    maintainer_email='trymva@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'task_manager_node = system_integration.task_manager_node:main',
    ],
    },
)
