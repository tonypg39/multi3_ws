from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

launch_args = [
    DeclareLaunchArgument(name='test_id', default_value='test_2_2', description='description'),
    DeclareLaunchArgument('mode',default_value='multi3'),
    DeclareLaunchArgument('sample_id',default_value='s0'),
    DeclareLaunchArgument('tbot_mapping',default_value=''),
    DeclareLaunchArgument('running_mode',default_value='virtual')
]

def launch_setup(context):
    test_id_value = LaunchConfiguration('test_id').perform(context) # Here you'll get the runtime config value
    tbot_mapping = LaunchConfiguration('tbot_mapping').perform(context) # Here you'll get the runtime config value
    # mode = LaunchConfiguration('config').perform(context)
    coordinator_node = Node(
        package='multi3_coordinator',
        executable='coordinator',
        name='coordinator',
        output='screen',
        parameters=[{'test_id': LaunchConfiguration("test_id"),'mode': LaunchConfiguration("mode")}],  # Pass the test_id parameter to the coordinator
    )
    n_execs = int(test_id_value.split("_")[1])
    executor_nodes = [
        Node(
            package='multi3_executor',
            executable='executor',
            name=f'robot_{i}',
            output='screen',
            parameters=[
                {'name': f'robot_{i}'},
                {'mode': LaunchConfiguration("running_mode")},
                {'test_id': LaunchConfiguration("test_id")},
                {'sample_id': LaunchConfiguration("sample_id")},
                {'tbot_mapping': LaunchConfiguration("tbot_mapping")}  # Pass the test_id parameter to each node
            ],
        )
        for i in range(1,n_execs+1)  # Convert to int at runtime
    ]
    return [coordinator_node] + executor_nodes

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld