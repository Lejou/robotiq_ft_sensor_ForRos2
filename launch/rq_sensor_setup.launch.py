from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
	serial_id = LaunchConfiguration('serial_id')
	serial_id_arg = DeclareLaunchArgument(
	  'serial_id', default_value='ttyUSB0'
   	)	
	
	rq_sensor_node =Node(
		package='robotiq_ft_sensor',
		executable='rq_sensor',
		name='rq_sensor',
		parameters=[{'serial_id': serial_id}]
	)
	ld = LaunchDescription() 
	ld.add_action(serial_id_arg)
	ld.add_action(rq_sensor_node)
	return ld