from setuptools import setup
from glob import glob
import os

package_name = 'rviz_visualizer'

# 遍历 urdf/leap_hand 及其子目录中的所有资源
urdf_files = [
    (os.path.join('share', package_name, os.path.dirname(path)), [path])
    for ext in ['urdf', 'obj', 'mtl', 'glb']
    for path in glob(f'urdf/**/*.{ext}', recursive=True)
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz_config'),
         glob('rviz_config/*.rviz')),
        *urdf_files,  # 安装所有 URDF 和 mesh 文件
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Leap hand RViz visualization package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
