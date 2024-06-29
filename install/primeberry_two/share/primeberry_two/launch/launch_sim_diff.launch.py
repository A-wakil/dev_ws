import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='primeberry_two' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_diff.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'primeberry_two'],
                        output='screen')
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'ackermann_controllers.yaml')],
        output='screen'
    )

    # Joint state broadcaster spawner
    spawner_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Ackermann steering controller spawner
    spawner_ackermann_controller= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_controller'],
        output='screen'
    )

    #teleop_twist keyboard
    teleop_twist_keyboard = Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    name='teleop_twist_keyboard',
    output='screen',
    prefix='xterm -e',  # This opens a new terminal window for teleop
    parameters=[{'use_sim_time': True}],
    remappings=[('/cmd_vel', '/ackermann_controller/cmd_vel_unstamped')]
    )

    delayed_joint_state_spawner = TimerAction(
        period=10.0,  # Delay in seconds
        actions=[spawner_joint_state_controller]
    )

    delayed_ackermann_spawner = TimerAction(
        period=15.0,  # Delay in seconds
        actions=[spawner_ackermann_controller]
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        controller_manager,
        delayed_joint_state_spawner,
        delayed_ackermann_spawner,
        teleop_twist_keyboard,
    ])