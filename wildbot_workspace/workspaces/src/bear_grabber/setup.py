from setuptools import find_packages, setup

package_name = 'bear_grabber'

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
    maintainer='組員 A',
    maintainer_email='your@email.com',
    description='平地夾熊並帶回基地的任務節點',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bear_grabber_node = bear_grabber.grab_and_return_node:main',
        ],
    },
)
