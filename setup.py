from setuptools import setup

package_name = 'vibelab_autoracer'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autoracer = vibelab_autoracer.autoracer:main',
            'csicam_publisher = vibelab_autoracer.csicam_publisher:main',
            'gstcam_publisher = vibelab_autoracer.gstcam_publisher:main',
            'cmd_vel_servo = vibelab_autoracer.cmd_vel_servo:main',
        ],
    },
)
