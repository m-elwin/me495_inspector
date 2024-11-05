from setuptools import find_packages, setup

package_name = 'inspector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/inspector.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elwin',
    maintainer_email='elwin@northwestern.edu',
    description='Intercept move_group actions from rviz to see what is happening.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspector = inspector.inspector:main'
        ],
    },
)
