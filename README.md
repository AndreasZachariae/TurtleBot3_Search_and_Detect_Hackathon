# Behavior Tree Coordinator
This package uses the IRAS/Common/BehaviorTree.ROS package which is a wrapper around the [BehaviorTree.cpp](https://www.behaviortree.dev/) package.

The Coordinator defines the behavior trees with single BT actions.

The set of actions can be arranged freely with the graphical user interface [Groot](https://github.com/BehaviorTree/Groot).

![example_tree](https://d33wubrfki0l68.cloudfront.net/f8b2bac65168251a46ec25232f20db7961327ffc/88ad1/images/readthedocs.png)

![groot](https://github.com/BehaviorTree/Groot/raw/master/groot-screenshot.png)

## How to start

Start the docker container

    source start_docker.sh

Compile mounted package

    colcon build
    source install/setup.bash

Attach a new docker shell

    docker exec -it coordinator bash

Launch the Coordinator node with parameters

    ros2 launch behavior_tree_coordinator demo.launch.py

Launch the Turtlebot3 Simulation (in a new docker shell)

    ros2 launch nav2_bringup tb3_simulation_launch.py
    
To view or modify the behavior trees (in a new docker shell)

    ros2 run groot Groot

Start Keyboard node for giving input (in a new docker shell)

    ros2 run petra_communication Keyboard

## Testing

    ros2 topic pub --once /Stop std_msgs/msg/Empty
    ros2 topic pub --once /SetBattery std_msgs/msg/Float32 "data: 0.1"
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: "map"}, pose: {position: {x: -2}}}"
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: "map"}, pose: {position: {x: 2}}}}"

