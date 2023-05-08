% Swarm Robotics, Spring 2023
% Doug de Jesus
% drd8913@nyu.edu
% N14928011


Nodes:
  soccer_sim: Node for the simulation environment. A heavily modified turtlesim_node.
    Services:
      Server:
        /kill
        /reset
        /spawn
        /<agent>/teleport_absolute
    Topics:
      Pub: 
        /<agent>/pose
      Sub: 
        /<agent>/cmd_vel
  player_formation_ctrl: Controller for the soccer player bots. Implements a decentralized formation control algorithm.
    Topics:
        Pub: 
          /<agent>/cmd_vel
    Services:
      Client:
        /get_distance
  distance_sensor: Service to get the distance between two agents (players and/or ball). To avoid redundant subscriptions, there is a single instance of this, rather than one for each agent.
    Topics:
      Sub: 
        /<agent>/pose
    Services:
      Server:
        /get_distance
  collision_detection: Detects collisions and updates the ball's velocity accordingly.
    Topics:
      Sub: 
        /<agent>/pose