from setuptools import setup

package_name = 'alarm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Alarm node package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alarm_node = alarm_pkg.alarm_node:main',
        ],
    },
)

