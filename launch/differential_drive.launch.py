from launch import LaunchDescription
import launch.actions 
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.actions
import ament_index_python.packages
import os

def generate_launch_description():
	joy_config = launch.substitutions.LaunchConfiguration('joy_config')
	joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
	config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')
	
                                           

	ld = LaunchDescription([
			launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        	launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        	launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                ament_index_python.packages.get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')])
            ])
	
	ld.add_action(launch_ros.actions.Node(
			package='joy',
			executable='joy_node',
			name='joy_node',
			output='both',
			parameters=[{
				'dev': joy_dev,
				'deadzone': 0.1,
				'autorepeat_rate': 20.0,}]))
				
	ld.add_action(launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath]))
	
	ld.add_action(launch_ros.actions.Node(
            name='teleop_drive',
            package='drive_base',
            executable='drive_base',
            output='screen',
			parameters=[
				'omnidirectional': false
			]))
           
	return ld

