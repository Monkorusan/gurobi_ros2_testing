from setuptools import setup

package_name = 'gurobi_ros2_testing'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'gurobipy', 'rclpy'],
    zip_safe=True,
    maintainer='monkorusan',
    maintainer_email='mongkoulchhuon@gmail.com',
    description='A test ROS2 node using Gurobi optimizer',
    license='MIT',
    entry_points={
        'console_scripts': [
            'qp_solver = gurobi_ros2_testing.qp_solver:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gurobi_qp.launch.py']),
        ('share/' + package_name + '/config', ['config/param.yaml']),
    ],
)

