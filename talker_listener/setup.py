from setuptools import setup, find_packages

package_name = 'talker_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/talker_listener.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sskspy',
    maintainer_email='shrungalkulkarni30@gmail.com',
    description='Minimal talker/listener example for ROS2 (ament_python)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = talker_listener.talker:main',
            'listener = talker_listener.listener:main',
        ],
    },
)
