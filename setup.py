from glob import glob

from setuptools import find_packages, setup

package_name = 'lerobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf/so101', glob('urdf/so101/*.urdf')),
        ('share/' + package_name + '/meshes/so101', glob('meshes/so101/*.stl')),
        ('share/' + package_name + '/mujoco/so101', glob('mujoco/so101/*.xml')),
        ('share/' + package_name + '/data', glob('data/*.json')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'st3215',
    ],
    zip_safe=True,
    maintainer='spenta',
    maintainer_email='spentazhou@gmail.com',
    description='一个基于so_arm101的机械臂视觉抓取项目',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
            'st3215',
        ],
    },
    entry_points={
        'console_scripts': [
            'red_block_detector_node = '
            'lerobot.vision.red_block_detector_node:main',
            'so101_joint_state_publisher_node = '
            'lerobot.so101.joint_state_publisher_node:main',
            'red_block_grasp_controller_node = '
            'lerobot.grasp.red_block_grasp_controller_node:main',
            'so101_sim_grasp_demo_node = '
            'lerobot.grasp.sim_grasp_demo_node:main',
        ],
    },
)
