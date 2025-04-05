import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_drive',
            executable='joy_node',
            name='joy_node'),
        launch_ros.actions.Node(
            package='robot_drive',
            executable='pwm',
            name='pwm_conversion'),
         launch_ros.actions.Node(
            package='robot_drive',
            executable='send_power',
            name='send_power'),
    ])