from setuptools import find_packages, setup

package_name = 'bracketbot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'motor_dir.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gongster',
    maintainer_email='gong.steven@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bracketbot_driver_node = bracketbot_driver.bracketbot_driver_node:main',
            'lqr_balance_with_input_node = bracketbot_driver.bracketbot_driver.lqr_balance_with_input_node:main'
        ],
    },
)
