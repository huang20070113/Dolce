from glob import glob
from setuptools import setup
setup(name='rm_enemy_pursuit', version='0.1.0', packages=['rm_enemy_pursuit'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rm_enemy_pursuit']),
                  ('share/rm_enemy_pursuit', ['package.xml', 'README.md', 'LICENSE']),
                  ('share/rm_enemy_pursuit/launch', glob('launch/*.launch.py')),
                  ('share/rm_enemy_pursuit/config', glob('config/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='huang20070113', maintainer_email='2817635381@qq.com',
      description='Priority pursuit using Nav2 actions', license='MIT',
      entry_points={'console_scripts': [
          'pursuit_manager = rm_enemy_pursuit.manager:main',
          'enemy_simulator = rm_enemy_pursuit.simulator:main']})
