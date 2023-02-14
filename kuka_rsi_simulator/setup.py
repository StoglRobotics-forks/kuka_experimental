from setuptools import setup

package_name = 'kuka_rsi_simulator'

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
    maintainer='Denis Štogl',
    maintainer_email='denis@stoglrobotics.de',
    description='Python node that implements a minimal RSI interface simulator.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	          'kuka_rsi_simulator = kuka_rsi_simulator.kuka_rsi_simulator:main',
        ],
    },
)
