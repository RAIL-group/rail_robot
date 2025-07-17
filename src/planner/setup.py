from setuptools import find_packages, setup

package_name = 'planner'

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
    maintainer='abhish',
    maintainer_email='abheeshkhanal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_planner_node = planner.base_planner:main',
            'mr_task_planner_node = planner.base_task_planner:main',
            'robot_pose_node = planner.robot_pose:main',
            'image_saver_node = planner.image_saver:main',
            'test_object_detection_node = planner.test_object_detection:main',
        ],
    },
)
