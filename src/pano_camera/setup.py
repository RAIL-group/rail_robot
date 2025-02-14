from setuptools import find_packages, setup

package_name = 'pano_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/pano_camera/launch', ['launch/pano_camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='railrobot',
    maintainer_email='railrobot@railrobot.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pano_camera_node = pano_camera.pano_camera_node:main'
        ],
    },
)
