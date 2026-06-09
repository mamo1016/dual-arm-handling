from glob import glob

from setuptools import find_packages, setup

package_name = 'dual_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'openarm_single_control.urdf']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mamoru Ueda',
    maintainer_email='27567543+mamo1016@users.noreply.github.com',
    description='Dual-arm 7-DOF manipulation demo in ROS 2: IK, singularity-robust '
                'Cartesian control, and a bimanual "hot potato" handoff.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'arm_planner = dual_arm.arm_planner:main',
            'joint_state_translator = dual_arm.joint_state_translator:main',
            'hot_potato = dual_arm.hot_potato:main',
        ],
    },
)
