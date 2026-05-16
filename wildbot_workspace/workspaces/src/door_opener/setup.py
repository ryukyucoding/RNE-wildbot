from setuptools import find_packages, setup

package_name = 'door_opener'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='組員 C',
    maintainer_email='your@email.com',
    description='開門任務節點',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_opener_node = door_opener.door_opener_node:main',
        ],
    },
)
