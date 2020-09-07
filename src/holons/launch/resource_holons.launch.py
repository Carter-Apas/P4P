import launch
import launch.actions
import launch.substitutions
import launch_ros.actions




def generate_launch_description():

    id = launch.LaunchDescription()
    id.add_action(launch_ros.actions.Node(package='holons', executable='R_KR16', output='screen',))
    id.add_action(launch_ros.actions.Node(package='holons', executable='R_KR10', output='screen',))
    id.add_action(launch_ros.actions.Node(package='holons', executable='R_CNC', output='screen',))
    id.add_action(launch_ros.actions.Node(package='holons', executable='R_CircConveyor', output='screen',))
    id.add_action(launch_ros.actions.Node(package='holons', executable='R_LinearConveyor', output='screen',))
    id.add_action(launch_ros.actions.Node(package='holons', executable='R_BasketA', output='screen',))
    id.add_action(launch_ros.actions.Node(package='holons', executable='R_BasketB', output='screen',))
    return id


        # launch_ros.actions.Node(
        #     package='holons', executable='R_KR10', output='screen',
        #     name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'R_KR10']),
        # launch_ros.actions.Node(
        #     package='holons', executable='R_BasketA', output='screen',
        #     name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'R_BasketA']),
        # launch_ros.actions.Node(
        #     package='holons', executable='R_BasketB', output='screen',
        #     name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'R_BasketB']),
        # launch_ros.actions.Node(
        #     package='holons', executable='R_CNC', output='screen',
        #     name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'R_CNC']),
        # launch_ros.actions.Node(
        #     package='holons', executable='R_CircConveyor', output='screen',
        #     name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'R_CircConveyor']),
        # launch_ros.actions.Node(
        #     package='holons', executable='R_LinearConveyor', output='screen',
        #     name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'R_LinearConveyor']),
