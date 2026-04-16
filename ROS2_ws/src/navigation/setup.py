from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigation'

# 递归获取目录下的所有文件
def get_data_files(data_dir, install_dir=None):
    if install_dir is None:
        install_dir = os.path.join('share', package_name, data_dir)
    
    data_files = []
    for root, dirs, files in os.walk(data_dir):
        for file in files:
            source_path = os.path.join(root, file)
            dest_dir = os.path.join(install_dir, os.path.relpath(root, data_dir))
            data_files.append((dest_dir, [source_path]))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'nav2_bringup'), glob('launch/nav2_bringup/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ]+ get_data_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjrgfdy',
    maintainer_email='3267512670@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_subscriber = navigation.uav_subscriber:main',
            'uav_manager = navigation.uav_manager:main',
            'uav_px4_fly_to_point = navigation.uav_px4_fly_to_point:main',             
        ],
    },
)