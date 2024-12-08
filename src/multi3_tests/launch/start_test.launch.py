from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Number of executor nodes to create
    n_executors = 2  # Adjust this number as needed

    # Create executor nodes with unique name parameters
    executor_nodes = [
        Node(
            package='multi3_executor',
            executable='executor',
            name=f'robot_{i}',
            output='screen',
            parameters=[{'name': f'robot_{i}'}],
        )
        for i in range(1,n_executors+1)
    ]

    # Create the coordinator node
    coordinator_node = Node(
        package='multi3_coordinator',
        executable='coordinator',
        name='coordinator',
        output='screen',
    )

    # Combine all nodes into a single launch description
    return LaunchDescription(executor_nodes + [coordinator_node])
