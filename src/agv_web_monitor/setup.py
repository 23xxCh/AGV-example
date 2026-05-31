from setuptools import setup

package_name = 'agv_web_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/web_monitor.launch.py']),
        ('share/' + package_name + '/templates', ['templates/dashboard.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Developer',
    maintainer_email='user@example.com',
    description='AGV Web监控面板',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_monitor_node = agv_web_monitor.web_monitor_node:main',
        ],
    },
)
