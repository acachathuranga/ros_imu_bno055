from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declare_arg_serial_port = DeclareLaunchArgument('serial_port', default_value='/dev/bno055', description='Device serial port')
    declare_arg_frame_id = DeclareLaunchArgument('frame_id', default_value='imu_link')
    declare_arg_frequency = DeclareLaunchArgument('frequency', default_value='20.0')
    declare_arg_namespace = DeclareLaunchArgument('namespace', default_value='robot')

    # Create Launch configurations
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')
    frequency = LaunchConfiguration('frequency')
    namespace = LaunchConfiguration('namespace')
    
    # Topic remappings
    # remappings = [('/map', 'map'), 
    #                 ('/tf', 'tf')]

    # Declare launch actions
    start_imu_node = Node(
        package='ros_imu_bno055',
        executable='imu_bno055',
        name='imu_bno055',
        namespace=namespace,
        output='screen',
        respawn = True,
        # remappings=remappings,
        parameters=[
        #   ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)
            {'serial_port': serial_port},
            {'frame_id': frame_id},
            {'frequency': frequency}],
        emulate_tty=True)
    
    start_imu_static_tf_publisher = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        name = "imu_tf_publisher",
        namespace=namespace,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        arguments = "0.025 0 0 0 0 0 base_link imu_link".split(' '))
 
    # Create Launch description
    ld = LaunchDescription()
    
    ld.add_action(declare_arg_serial_port)
    ld.add_action(declare_arg_frame_id)
    ld.add_action(declare_arg_frequency)
    ld.add_action(declare_arg_namespace)

    ld.add_action(start_imu_node)
    ld.add_action(start_imu_static_tf_publisher)
    return ld