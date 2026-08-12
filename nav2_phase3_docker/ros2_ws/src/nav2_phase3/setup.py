from setuptools import setup

package_name = 'nav2_phase3'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', ['launch/phase3.launch.py']),
                  ('share/' + package_name + '/config', ['config/phase3.yaml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['mission_manager = nav2_phase3.mission_manager:main',
                                        'ball_detector = nav2_phase3.ball_detector:main',
                                        'ball_approach_manager = nav2_phase3.ball_approach_manager:main']})
