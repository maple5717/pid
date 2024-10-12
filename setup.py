from setuptools import setup

package_name = 'pid'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='License of your choice',
    entry_points={
        'console_scripts': [
            'wheelchair_pid_helper = pid.wheelchair_pid_helper:main',
        ],
    },
)
