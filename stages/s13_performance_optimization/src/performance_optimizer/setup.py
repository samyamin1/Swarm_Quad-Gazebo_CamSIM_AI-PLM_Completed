from setuptools import setup

package_name = 'performance_optimizer'

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
    description='Performance Optimization for AI-powered swarm operations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'performance_optimizer = performance_optimizer.performance_optimizer:main',
        ],
    },
) 