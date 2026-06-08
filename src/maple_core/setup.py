from setuptools import find_packages, setup

package_name = 'maple_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='shivamwalia2006@gmail.com',
    description='Maple orchestration node: drives the PyLips face and publishes motion commands.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'maple_orchestrator = maple_core.maple_orchestrator:main',
        ],
    },
)
