from setuptools import find_packages, setup

package_name = 'py_pubsub_imgs'

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
    maintainer='xiang-tao',
    maintainer_email='xiang.tao@outlook.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'multi_listener = py_pubsub_imgs.multi_listener:main',
                'multi_listener_sync = py_pubsub_imgs.multi_listener_sync:main',
                'single_listener = py_pubsub_imgs.single_listener:main',
                'single_lidar = py_pubsub_imgs.single_lidar:main',
                'single_imu = py_pubsub_imgs.single_imu:main',
                'sync_analysis = py_pubsub_imgs.sync_analysis:main',
                'sync_data = py_pubsub_imgs.sync_data:main',
        ],
},
)
