from setuptools import setup

package_name = 'drone_racer'

data_files = []
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/drone_racer_irl_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/drone_racer_simulation_launch.py']))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nevil',
    maintainer_email='nevil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_control = drone_racer.drone_control_sim:main',
            'gate_finder = drone_racer.gate_finder_sim:main',
            'monitor = drone_racer.monitor_sim:main',
            'drone_control_irl = drone_racer.drone_control_irl:main',
            'gate_finder_irl = drone_racer.gate_finder_irl:main',
            'monitor_irl = drone_racer.monitor_irl:main',
        ],
    },
)
