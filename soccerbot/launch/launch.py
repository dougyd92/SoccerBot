from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='soccer_sim', executable='soccer_sim_node'),
        Node(
            package='soccerbot', executable='distance_sensor'),
        Node(
            package='soccerbot', executable='collision_detection'),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 34, y: 52.5, agent_type: BALL, name: ball}"',
          ]],
          shell=True
        ),
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single_agent_launch.py']),
          launch_arguments={
              'agent_type': 'TEAM_A',
              'agent_name': 'player_A1',
              'spawn_position': 'x: 16, y: 10'
          }.items()
        ),
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single_agent_launch.py']),
          launch_arguments={
              'agent_type': 'TEAM_A',
              'agent_name': 'player_A2',
              'spawn_position': 'x: 32, y: 10'
          }.items()
        ),
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single_agent_launch.py']),
          launch_arguments={
              'agent_type': 'TEAM_A',
              'agent_name': 'player_A3',
              'spawn_position': 'x: 48, y: 10'
          }.items()
        ),
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single_agent_launch.py']),
          launch_arguments={
              'agent_type': 'TEAM_B',
              'agent_name': 'player_B1',
              'spawn_position': 'x: 16, y: 95'
          }.items()
        ),
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single_agent_launch.py']),
          launch_arguments={
              'agent_type': 'TEAM_B',
              'agent_name': 'player_B2',
              'spawn_position': 'x: 32, y: 95'
          }.items()
        ),
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single_agent_launch.py']),
          launch_arguments={
              'agent_type': 'TEAM_B',
              'agent_name': 'player_B3',
              'spawn_position': 'x: 48, y: 95'
          }.items()
        )   
    ])
