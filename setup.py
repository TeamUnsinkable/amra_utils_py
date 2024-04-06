from setuptools import find_packages, setup
from glob import glob

package_name = 'amra_utils_py'

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
    maintainer='robosub',
    maintainer_email='siriojansen@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_translator = amra_utils_py.joystick_translator:main",
            "joy_actuator = amra_utils_py.joystick_actuator:main",
            "vectornav_translator = amra_utils_py.vectornav_translator:main"
        ],
    },
)
