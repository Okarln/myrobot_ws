from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'myrobot_real'

setup(
    name=package_name,
    version='1.0.0',
    packages=['myrobot_real'],
    package_dir={'myrobot_real': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='legolas',
    maintainer_email='legolas@todo.todo',
    description='实车集成包:轮速桥接 + 麦轮运动学 + EKF/Nav2 实车 launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_bridge = myrobot_real.wheel_bridge:main',
            'wheel_kinematics = myrobot_real.wheel_kinematics:main',
            'mock_stm32 = myrobot_real.mock_stm32:main',
            'wheel_calibration = myrobot_real.wheel_calibration:main',
            'wheel_log = myrobot_real.wheel_log:main',
            'map_tf_adjust = myrobot_real.map_tf_adjust:main',
            'push_odom = myrobot_real.push_odom:main',
        ],
    },
)
