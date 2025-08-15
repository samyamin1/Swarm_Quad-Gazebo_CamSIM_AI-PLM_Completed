from setuptools import setup

package_name = 'full_system_integration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Swarm Team',
    maintainer_email='developer@example.com',
    description='Full System Integration for AI-powered swarm quadcopter simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_integrator = full_system_integration.system_integrator:main',
        ],
    },
) 