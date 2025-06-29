from setuptools import setup, find_packages

package_name = 'lane_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # src/ 없이도 find_packages()로 lane_follower 모듈 인식
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/test', ['test/lanevideo.mp4']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gyuhyeok',
    maintainer_email='gyuhyeok@example.com',
    description='Lane following robot using ROS2 and OpenCV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_follower_node = lane_follower.lane_follower_node:main',
        ],
    },
)
