from setuptools import setup, find_packages

package_name = 'ekf_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='support@example.com',
    description='Modular 2D land-vehicle EKF workspace for GNSS/IMU/wheel-speed/constraint fusion.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_simulation = ekf_project.runners.run_simulation:main',
            'run_log_replay = ekf_project.runners.run_log_replay:main',
            'live_rmse_plotter = ekf_project.analysis.live_rmse_plotter:main',
        ],
    },
)
