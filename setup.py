from setuptools import setup

package_name = 'traj_gen'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Camilla Tabasso',
    maintainer_email='camilla-tabasso@uiowa.edu',
    description='Trajectory handler for Crazyflie swarm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_points_pub = traj_gen.control_points:main',
            'traj_cp = traj_gen.traj_evaluate_cp:main',
            'traj_xyz = traj_gen.traj_evaluate_xyz:main',
            'traj_vicon = traj_gen.traj_evaluate_vicon:main',
        ],
    },
)
