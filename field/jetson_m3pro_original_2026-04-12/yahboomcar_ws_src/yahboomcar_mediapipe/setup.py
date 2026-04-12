from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_mediapipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share','yahboomcar_mediapipe','rviz'),glob(os.path.join('rviz','*.rviz*'))),
    (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='13377528435@sina.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        '01_HandDetector = yahboomcar_mediapipe.01_HandDetector:main',
        '02_PoseDetector = yahboomcar_mediapipe.02_PoseDetector:main',
        '03_Holistic = yahboomcar_mediapipe.03_Holistic:main',
        '04_FaceMesh = yahboomcar_mediapipe.04_FaceMesh:main',
        '05_FaceEyeDetection = yahboomcar_mediapipe.05_FaceEyeDetection:main',
		'06_FaceLandmarks = yahboomcar_mediapipe.06_FaceLandmarks:main',	
		'07_FaceDetection = yahboomcar_mediapipe.07_FaceDetection:main',
		'08_Objectron  = yahboomcar_mediapipe.08_Objectron:main',
		'09_VirtualPaint = yahboomcar_mediapipe.09_VirtualPaint:main',
		'10_HandCtrl = yahboomcar_mediapipe.10_HandCtrl:main',
		'11_GestureRecognition = yahboomcar_mediapipe.11_GestureRecognition:main',
		'12_FindHand = yahboomcar_mediapipe.12_FindHand:main',
		'13_FingerCtrl = yahboomcar_mediapipe.13_FingerCtrl:main',
		'14_FingerAction = yahboomcar_mediapipe.14_FingerAction:main',
		'15_FingerTrajectory = yahboomcar_mediapipe.15_FingerTrajectory:main',
		'16_GestureGrasp = yahboomcar_mediapipe.16_GestureGrasp:main',
        'test_msg = yahboomcar_mediapipe.test_msg:main',
        'finger_arm = yahboomcar_mediapipe.finger_arm:main'
        ],
    },
)
