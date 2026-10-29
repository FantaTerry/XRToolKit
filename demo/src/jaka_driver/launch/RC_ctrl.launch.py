import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    control_config_arg = DeclareLaunchArgument(
        'control_config',
        default_value=PathJoinSubstitution([FindPackageShare('jaka_driver'), 'config', 'teleop_control.yaml']),
        description='Servo timing, trajectory limits, freshness, filter and diagnostics YAML')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    # Declare launch arguments
    robot_left_ip_arg = DeclareLaunchArgument(
        'robot_left_ip',
        default_value='192.168.71.37',
        description='IP address of the left JAKA robot'
    )
    robot_right_ip_arg = DeclareLaunchArgument(
        'robot_right_ip',
        default_value='192.168.71.36',
        description='IP address of the right JAKA robot'
    )
    rc_ctrl_arg = DeclareLaunchArgument(
        'rc_ctrl',
        default_value='true',
        description='Enable or disable RC control'
    )
    left_target_topic_arg = DeclareLaunchArgument(
        'left_target_topic',
        default_value='/rc_ctrl/left_target',
        description='Topic carrying the left handle target pose (PoseStamped)'
    )
    right_target_topic_arg = DeclareLaunchArgument(
        'right_target_topic',
        default_value='/rc_ctrl/right_target',
        description='Topic carrying the right handle target pose (PoseStamped)'
    )
    button_topic_arg = DeclareLaunchArgument(
        'button_topic',
        default_value='/rc_ctrl/button',
        description='Topic carrying the VR button/grip state (sensor_msgs/Joy)'
    )
    gripper_force_arg = DeclareLaunchArgument(
        'gripper_force', default_value='20',
        description='PGI force percentage (20..100)')
    gripper_speed_arg = DeclareLaunchArgument(
        'gripper_speed', default_value='100',
        description='PGI speed percentage (1..100)')
    gripper_baudrate_arg = DeclareLaunchArgument(
        'gripper_baudrate', default_value='115200',
        description='PGI RS485 baud rate (default 115200)')
    gripper_rs485_channel_arg = DeclareLaunchArgument(
        'gripper_rs485_channel', default_value='1',
        description='JAKA SDK RS485 channel: 0=RS485H/TIO channel 1, 1=RS485L/TIO channel 2')
    gripper_slave_id_arg = DeclareLaunchArgument(
        'gripper_slave_id', default_value='1',
        description='PGI Modbus slave ID')
    gripper_open_position_arg = DeclareLaunchArgument(
        'gripper_open_position', default_value='1000',
        description='PGI open target in per mille (0..1000)')
    gripper_closed_position_arg = DeclareLaunchArgument(
        'gripper_closed_position', default_value='0',
        description='PGI closed target in per mille (0..1000)')
    gripper_linear_arg = DeclareLaunchArgument(
        'gripper_linear', default_value='true',
        description='Use linear (analog axes) control for gripper by default')
    gripper_trigger_deadband_arg = DeclareLaunchArgument(
        'gripper_trigger_deadband', default_value='0.02',
        description='Minimal change in analog trigger to send a new Grip()')
    gripper_initialize_arg = DeclareLaunchArgument(
        'gripper_initialize', default_value='false',
        description='Initialize PGI gripper during node startup')
    gripper_startup_open_arg = DeclareLaunchArgument('gripper_startup_open', default_value='false', description='Explicitly open on startup; false preserves held load')
    gripper_motion_parameters_arg = DeclareLaunchArgument('gripper_configure_motion_parameters', default_value='false', description='Explicit startup force/speed writes')
    gripper_initialize_command_arg = DeclareLaunchArgument(
        'gripper_initialize_command', default_value='1',
        description='PGI initialization command: 1 or 165 (0xA5)')
    gripper_initialize_delay_arg = DeclareLaunchArgument(
        'gripper_initialize_delay_ms', default_value='2000',
        description='Wait time after PGI initialization command')
    gripper_configure_tio_arg = DeclareLaunchArgument(
        'gripper_configure_tio', default_value='true',
        description='Configure TIO via SDK; false uses the JAKA App saved configuration')
    gripper_enable_tio_power_arg = DeclareLaunchArgument(
        'gripper_enable_tio_power', default_value='true',
        description='Enable JAKA TIO voltage output for the gripper')
    gripper_tio_voltage_arg = DeclareLaunchArgument(
        'gripper_tio_voltage', default_value='0',
        description='JAKA TIO output voltage: 0=24 V, 1=12 V')
    gripper_use_button_arg = DeclareLaunchArgument(
        'gripper_use_button', default_value='false',
        description='Use only Joy.buttons; default also accepts digital trigger fallback')
    gripper_button_index_arg = DeclareLaunchArgument(
        'gripper_button_index', default_value='-1',
        description='Joy.buttons index; -1 selects left LTr=10 or right RTr=11')
    use_rpy_ctrl_arg = DeclareLaunchArgument(
        'use_rpy_ctrl', default_value='true',
        description='true = 6D teleop including orientation; false = position only')
    rotation_scale_arg = DeclareLaunchArgument(
        'rotation_scale', default_value='0.5',
        description='Relative controller rotation mapped to robot orientation (0..1)')
    rotation_gain_arg = DeclareLaunchArgument(
        'rotation_gain', default_value='0.5',
        description='Fraction of remaining controller rotation applied per cycle (0..1)')
    rotation_deadband_arg = DeclareLaunchArgument(
        'rotation_deadband_rad', default_value='0.005',
        description='Ignore controller orientation changes below this angle in radians')
    rotation_max_step_arg = DeclareLaunchArgument(
        'rotation_max_step_rad', default_value='0.0015',
        description='Maximum commanded rotation per 8 ms servo cycle in radians')
    rotation_sign_arg = DeclareLaunchArgument(
        'rotation_sign', default_value='1',
        description='Robot rotation direction: 1 or -1, applied to all rotation axes')
    grip_press_threshold_arg = DeclareLaunchArgument(
        'grip_press_threshold', default_value='0.6',
        description='Grip value required to engage arm control')
    grip_release_threshold_arg = DeclareLaunchArgument(
        'grip_release_threshold', default_value='0.4',
        description='Grip value below which arm control is released')
    enable_y_limit_arg = DeclareLaunchArgument(
        'enable_y_limit', default_value='true',
        description='true = clamp arm TCP y (left > -110 mm, right < 110 mm)')

    # One process per arm: the JAKA SDK holds a single global connection per
    # process, so each rc_ctrl_node instance drives exactly one robot. Two
    # instances (left / right) are launched here, each logging into its own arm
    # and reading its own grip axis from the shared /rc_ctrl/button Joy topic.
    rc_ctrl_left_node = Node(
        package='jaka_driver',
        executable='rc_ctrl_node',
        name='rc_ctrl_left',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('rc_ctrl')),
        parameters=[LaunchConfiguration('control_config'), {
            'arm_name': 'left',
            'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
            'robot_ip': LaunchConfiguration('robot_left_ip'),
            'target_topic': LaunchConfiguration('left_target_topic'),
            'button_topic': LaunchConfiguration('button_topic'),
            'grip_axis': 0,
            'gripper_axis': 2,
            'gripper_force': ParameterValue(LaunchConfiguration('gripper_force'), value_type=int),
            'gripper_speed': ParameterValue(LaunchConfiguration('gripper_speed'), value_type=int),
            'gripper_baudrate': ParameterValue(LaunchConfiguration('gripper_baudrate'), value_type=int),
            'gripper_rs485_channel': ParameterValue(LaunchConfiguration('gripper_rs485_channel'), value_type=int),
            'gripper_slave_id': ParameterValue(LaunchConfiguration('gripper_slave_id'), value_type=int),
            'gripper_open_position': ParameterValue(LaunchConfiguration('gripper_open_position'), value_type=int),
            'gripper_closed_position': ParameterValue(LaunchConfiguration('gripper_closed_position'), value_type=int),
            'gripper_linear': ParameterValue(LaunchConfiguration('gripper_linear'), value_type=bool),
            'gripper_trigger_deadband': ParameterValue(LaunchConfiguration('gripper_trigger_deadband'), value_type=float),
            'gripper_initialize': ParameterValue(LaunchConfiguration('gripper_initialize'), value_type=bool),
            'gripper_startup_open': ParameterValue(LaunchConfiguration('gripper_startup_open'), value_type=bool),
            'gripper_configure_motion_parameters': ParameterValue(LaunchConfiguration('gripper_configure_motion_parameters'), value_type=bool),
            'gripper_initialize_command': ParameterValue(LaunchConfiguration('gripper_initialize_command'), value_type=int),
            'gripper_initialize_delay_ms': ParameterValue(LaunchConfiguration('gripper_initialize_delay_ms'), value_type=int),
            'gripper_configure_tio': ParameterValue(LaunchConfiguration('gripper_configure_tio'), value_type=bool),
            'gripper_enable_tio_power': ParameterValue(LaunchConfiguration('gripper_enable_tio_power'), value_type=bool),
            'gripper_tio_voltage': ParameterValue(LaunchConfiguration('gripper_tio_voltage'), value_type=int),
            'gripper_use_button': ParameterValue(LaunchConfiguration('gripper_use_button'), value_type=bool),
            'gripper_button_index': ParameterValue(LaunchConfiguration('gripper_button_index'), value_type=int),
            'use_rpy_ctrl': ParameterValue(LaunchConfiguration('use_rpy_ctrl'), value_type=bool),
            'rotation_scale': ParameterValue(LaunchConfiguration('rotation_scale'), value_type=float),
            'rotation_gain': ParameterValue(LaunchConfiguration('rotation_gain'), value_type=float),
            'rotation_deadband_rad': ParameterValue(LaunchConfiguration('rotation_deadband_rad'), value_type=float),
            'rotation_max_step_rad': ParameterValue(LaunchConfiguration('rotation_max_step_rad'), value_type=float),
            'rotation_sign': ParameterValue(LaunchConfiguration('rotation_sign'), value_type=int),
            'grip_press_threshold': ParameterValue(LaunchConfiguration('grip_press_threshold'), value_type=float),
            'grip_release_threshold': ParameterValue(LaunchConfiguration('grip_release_threshold'), value_type=float),
            'enable_y_limit': ParameterValue(LaunchConfiguration('enable_y_limit'), value_type=bool),
        }]
    )
    rc_ctrl_right_node = Node(
        package='jaka_driver',
        executable='rc_ctrl_node',
        name='rc_ctrl_right',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('rc_ctrl')),
        parameters=[LaunchConfiguration('control_config'), {
            'arm_name': 'right',
            'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
            'robot_ip': LaunchConfiguration('robot_right_ip'),
            'target_topic': LaunchConfiguration('right_target_topic'),
            'button_topic': LaunchConfiguration('button_topic'),
            'grip_axis': 1,
            'gripper_axis': 3,
            'gripper_force': ParameterValue(LaunchConfiguration('gripper_force'), value_type=int),
            'gripper_speed': ParameterValue(LaunchConfiguration('gripper_speed'), value_type=int),
            'gripper_baudrate': ParameterValue(LaunchConfiguration('gripper_baudrate'), value_type=int),
            'gripper_rs485_channel': ParameterValue(LaunchConfiguration('gripper_rs485_channel'), value_type=int),
            'gripper_slave_id': ParameterValue(LaunchConfiguration('gripper_slave_id'), value_type=int),
            'gripper_open_position': ParameterValue(LaunchConfiguration('gripper_open_position'), value_type=int),
            'gripper_closed_position': ParameterValue(LaunchConfiguration('gripper_closed_position'), value_type=int),
            'gripper_linear': ParameterValue(LaunchConfiguration('gripper_linear'), value_type=bool),
            'gripper_trigger_deadband': ParameterValue(LaunchConfiguration('gripper_trigger_deadband'), value_type=float),
            'gripper_initialize': ParameterValue(LaunchConfiguration('gripper_initialize'), value_type=bool),
            'gripper_startup_open': ParameterValue(LaunchConfiguration('gripper_startup_open'), value_type=bool),
            'gripper_configure_motion_parameters': ParameterValue(LaunchConfiguration('gripper_configure_motion_parameters'), value_type=bool),
            'gripper_initialize_command': ParameterValue(LaunchConfiguration('gripper_initialize_command'), value_type=int),
            'gripper_initialize_delay_ms': ParameterValue(LaunchConfiguration('gripper_initialize_delay_ms'), value_type=int),
            'gripper_configure_tio': ParameterValue(LaunchConfiguration('gripper_configure_tio'), value_type=bool),
            'gripper_enable_tio_power': ParameterValue(LaunchConfiguration('gripper_enable_tio_power'), value_type=bool),
            'gripper_tio_voltage': ParameterValue(LaunchConfiguration('gripper_tio_voltage'), value_type=int),
            'gripper_use_button': ParameterValue(LaunchConfiguration('gripper_use_button'), value_type=bool),
            'gripper_button_index': ParameterValue(LaunchConfiguration('gripper_button_index'), value_type=int),
            'use_rpy_ctrl': ParameterValue(LaunchConfiguration('use_rpy_ctrl'), value_type=bool),
            'rotation_scale': ParameterValue(LaunchConfiguration('rotation_scale'), value_type=float),
            'rotation_gain': ParameterValue(LaunchConfiguration('rotation_gain'), value_type=float),
            'rotation_deadband_rad': ParameterValue(LaunchConfiguration('rotation_deadband_rad'), value_type=float),
            'rotation_max_step_rad': ParameterValue(LaunchConfiguration('rotation_max_step_rad'), value_type=float),
            'rotation_sign': ParameterValue(LaunchConfiguration('rotation_sign'), value_type=int),
            'grip_press_threshold': ParameterValue(LaunchConfiguration('grip_press_threshold'), value_type=float),
            'grip_release_threshold': ParameterValue(LaunchConfiguration('grip_release_threshold'), value_type=float),
            'enable_y_limit': ParameterValue(LaunchConfiguration('enable_y_limit'), value_type=bool),
        }]
    )

    return LaunchDescription([
        control_config_arg,
        debug_arg,
        robot_left_ip_arg,
        robot_right_ip_arg,
        rc_ctrl_arg,
        left_target_topic_arg,
        right_target_topic_arg,
        button_topic_arg,
        gripper_force_arg,
        gripper_speed_arg,
        gripper_baudrate_arg,
        gripper_rs485_channel_arg,
        gripper_slave_id_arg,
        gripper_open_position_arg,
        gripper_closed_position_arg,
        gripper_linear_arg,
        gripper_trigger_deadband_arg,
        gripper_initialize_arg,
        gripper_startup_open_arg,
        gripper_motion_parameters_arg,
        gripper_initialize_command_arg,
        gripper_initialize_delay_arg,
        gripper_configure_tio_arg,
        gripper_enable_tio_power_arg,
        gripper_tio_voltage_arg,
        gripper_use_button_arg,
        gripper_button_index_arg,
        use_rpy_ctrl_arg,
        rotation_scale_arg,
        rotation_gain_arg,
        rotation_deadband_arg,
        rotation_max_step_arg,
        rotation_sign_arg,
        grip_press_threshold_arg,
        grip_release_threshold_arg,
        enable_y_limit_arg,
        rc_ctrl_left_node,
        rc_ctrl_right_node,
        LogInfo(msg="RC control nodes launched.")
    ])
