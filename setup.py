from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'colav_risk_assessment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools',
                      'unsafe_set_gen'],
    zip_safe=True,
    maintainer='Ryan McKee',
    maintainer_email='r.mckee@qub.ac.uk',
    description=( 
        'COLAV risk assesment provides a number of different nodes for the ' 
        'colav system producing and listening to data related to risk assesment '
        ' for motion planning'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unsafe_set_generator_node = colav_risk_assessment.execute_colav_risk_assesment:main'
        ],
    },
)
