from setuptools import setup

setup(name='rc_car', version='0.1.0', packages=['rc_car'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rc_car']),
                  ('share/rc_car', ['package.xml']),
                  ('share/rc_car/launch', ['launch/vehicle.launch.py']),
                  ('share/rc_car/web', ['web/index.html', 'web/app.js', 'web/style.css'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'command_manager = rc_car.nodes:manager_main',
          'vehicle_controller = rc_car.nodes:controller_main',
          'odometry = rc_car.nodes:odometry_main',
          'web_bridge = rc_car.web:main']})
