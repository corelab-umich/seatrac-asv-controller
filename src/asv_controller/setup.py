from setuptools import find_packages, setup

package_name = 'asv_controller'

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
    maintainer='kmgovind',
    maintainer_email='kavin.gmk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'synthetic_data_pub = asv_controller.juliatest:main',
            'asv_ergo_control = asv_controller.AsvErgoControl:main',
            'param_estimator = asv_controller.ParamEstimator:main',
            'q_map_pub = asv_controller.QMapPub:main',
            'w_hat_pub = asv_controller.WHatPub:main',
            'target_q_map_pub = asv_controller.TargetQMapPub:main'
        ],
    },
)
