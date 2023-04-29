from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='soccer_sim', executable='turtlesim_node'),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 1, y: 1, name: player1}"',
          ]],
          shell=True
        ),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 2, y: 2, name: player2}"',
          ]],
          shell=True
        ),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 3, y: 3, name: player3}"',
          ]],
          shell=True
        ),
        Node(
            namespace='player1',
            package='soccerbot', executable='soccer_player', 
            remappings=[('/agent1/cmd_vel', '/player1/cmd_vel')]),
        Node(
            namespace='player2',
            package='soccerbot', executable='soccer_player', 
            remappings=[('/agent1/cmd_vel', '/player2/cmd_vel')]),
        Node(
            namespace='player3',
            package='soccerbot', executable='soccer_player', 
            remappings=[('/agent1/cmd_vel', '/player3/cmd_vel')]),
    ])
