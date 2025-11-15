from setuptools import setup
import os
from glob import glob


package_name = 'leap_hand_retargeting'

# 遍历 urdf/leap_hand 及其子目录中的所有资源
urdf_files = [
    (os.path.join('share', package_name, os.path.dirname(path)), [path])
    for ext in ['urdf', 'obj', 'mtl', 'glb']
    for path in glob(f'urdf/**/*.{ext}', recursive=True)
]
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *urdf_files,  # 安装所有 URDF 和 mesh 文件
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=[
    'setuptools',
    'ament_index_python',
    ],
    zip_safe=True,
    maintainer='Kay',
    maintainer_email='KyaKayKya@gmail.com',
    description='Leap Motion hand keypoint retargeting using URDF and Pinocchio.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_keypoint_retargeter = leap_hand_retargeting.hand_keypoint_retargeter:main',
        ],
    },
)
