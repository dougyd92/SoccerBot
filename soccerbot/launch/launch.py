from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='soccer_sim', executable='soccer_sim_node'),
        Node(
            package='soccerbot', executable='distance_sensor'),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 1, y: 1, agent_type: TEAM_A, name: player1}"',
          ]],
          shell=True
        ),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 2, y: 2, agent_type: TEAM_A, name: player2}"',
          ]],
          shell=True
        ),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 3, y: 3, agent_type: TEAM_B, name: player3}"',
          ]],
          shell=True
        ),
        ExecuteProcess(
          cmd=[[
            'ros2 service call ',
            '/spawn ',
            'soccer_sim/srv/Spawn ',
            '"{x: 5, y: 5, agent_type: BALL, name: ball}"',
          ]],
          shell=True
        ),
        Node(
            namespace='player1',
            package='soccerbot', executable='formation_controller', 
            remappings=[('/agent1/cmd_vel', '/player1/cmd_vel')]),
        Node(
            namespace='player2',
            package='soccerbot', executable='formation_controller', 
            remappings=[('/agent1/cmd_vel', '/player2/cmd_vel')]),
        Node(
            namespace='player3',
            package='soccerbot', executable='formation_controller', 
            remappings=[('/agent1/cmd_vel', '/player3/cmd_vel')]),
    ])
