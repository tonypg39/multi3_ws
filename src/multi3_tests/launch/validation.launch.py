from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    test_id = LaunchConfiguration('test_id')
    mode = LaunchConfiguration('mode')

    test_id_launch_arg = DeclareLaunchArgument(
        'test_id',
        default_value='_'
    )
    mode_launch_arg = DeclareLaunchArgument(
        'mode',
        default_value='multi3'
    )


    test_rt = test_id.perform(None)
    # Generate executor nodes dynamically
    executor_nodes = [
        Node(
            package='multi3_executor',
            executable='executor',
            name=f'robot_{i}',
            output='screen',
            parameters=[
                {'name': f'robot_{i}'},
                {'test_id': "test_id"},  # Pass the test_id parameter to each node
            ],
        )
        for i in range(2)  # Convert to int at runtime
    ]

    # Create the coordinator node
    coordinator_node = Node(
        package='multi3_coordinator',
        executable='coordinator',
        name='coordinator',
        output='screen',
        parameters=[{'test_id': test_id,'mode': mode}],  # Pass the test_id parameter to the coordinator
    )

    # Combine all nodes into a single launch description
    # return LaunchDescription([n_executors_arg, test_id_arg] + executor_nodes + [coordinator_node])
    return LaunchDescription([test_id_launch_arg, mode_launch_arg] + [coordinator_node])
