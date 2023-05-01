from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
  agent_type = LaunchConfiguration('agent_type')
  agent_name = LaunchConfiguration('agent_name')
  spawn_position = LaunchConfiguration('spawn_position')
  
  agent_type_launch_arg = DeclareLaunchArgument(
    'agent_type', default_value='TEAM_A'
  )
  agent_name_launch_arg = DeclareLaunchArgument(
    'agent_name', default_value='player1'
  )
  spawn_position_launch_arg = DeclareLaunchArgument(
    'spawn_position', default_value='x: 1, y: 1'
  )

  return LaunchDescription([
    agent_type_launch_arg,
    agent_name_launch_arg,
    spawn_position_launch_arg,
    ExecuteProcess(
      cmd=[[
        'ros2 service call ',
        '/spawn ',
        'soccer_sim/srv/Spawn ',
        '"{', spawn_position, ', agent_type: ', agent_type, ', name: ', agent_name, '}"',
      ]],
      shell=True
    ),
    Node(
      namespace=agent_name,
      package='soccerbot', executable='formation_controller', 
      remappings=[('/agent1/cmd_vel', PythonExpression(expression=["'/", agent_name, "/cmd_vel'"]) )]),
  ])