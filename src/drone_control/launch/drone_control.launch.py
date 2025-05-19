import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_control')
    world_path = os.path.expanduser(
        '~/ros2_ws/rl_drone_ws/src/drone_sim_world/worlds/drone_world.sdf'
    )

    px4 = ExecuteProcess(
        cmd=[
            './bin/px4',
            '-i', '0',
            '-d', 'etc',
            '-s', 'etc/init.d-posix/rcS'
        ],
        cwd=os.path.expanduser(
            '~/ros2_ws/rl_drone_ws/src/PX4-Autopilot/build/px4_sitl_default'
        ),
        output='screen'
    )

    xrce = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'udp4', '-p', '8888'
        ],
        cwd=os.path.expanduser(
            '~/tools/px4_agents/Micro-XRCE-DDS-Agent'
        ),
        output='screen'
    )

    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_path],
        output='screen'
    )

    spawn_drone = ExecuteProcess(
        cmd=[
            'ign', 'service', '-s', '/world/drone_test_world/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '3000',
            '--req', 'sdf_filename: "model://x500", name: "x500"'
        ],
        output='screen'
    )

    offboard_fsm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'offboard_fsm.launch.py')
        )
    )

    return LaunchDescription([
        px4,
        xrce,
        ign_gazebo,
        TimerAction(period=2.0, actions=[spawn_drone]),
        TimerAction(period=3.0, actions=[offboard_fsm]),
    ])
