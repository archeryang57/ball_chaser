from setuptools import setup

package_name = 'ball_chaser'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "process_image=ball_chaser.process_image:main",
            "process_image_cv=ball_chaser.process_image_cv:main",
            "estimate_color=ball_chaser.estimate_color:main",
            "drive_bot=ball_chaser.drive_bot:main",
            "test_ball=ball_chaser.test_ball:main",
            "range_detector=ball_chaser.range_detector:main"
        ],
    },
)
