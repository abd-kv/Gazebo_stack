from setuptools import setup

package_name = 'event_camera_sim_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simulates event camera from grayscale images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_to_events = event_camera_sim_py.image_to_events:main',
        ],
    },
)
