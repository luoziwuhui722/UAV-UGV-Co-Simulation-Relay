import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable   # 已存在
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    px4_dir = LaunchConfiguration('px4_dir').perform(context).strip()
    if not px4_dir:
        raise RuntimeError("'px4_dir' is empty. Set px4_dir or export PX4_AUTOPILOT_PATH.")
    if not os.path.isfile(os.path.join(px4_dir, 'Makefile')):
        raise RuntimeError(f"px4_dir '{px4_dir}' is invalid (missing Makefile).")

    current_file_dir = os.path.dirname(os.path.realpath(__file__))
    package_dir = os.path.dirname(current_file_dir)
    world_file = os.path.join(package_dir, 'worlds', 'Factory.world')
    if not os.path.isfile(world_file):
        raise RuntimeError(f"World file not found: {world_file}")

    px4_process = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gazebo-classic'],
        cwd=px4_dir,
        output='screen',
        emulate_tty=True,
    )

    return [
        LogInfo(msg=f'Starting PX4 SITL with Gazebo from directory: {px4_dir}'),
        LogInfo(msg=f'Using world file: {world_file}'),

        # 关闭视角跟随
        SetEnvironmentVariable(name='PX4_NO_FOLLOW_MODE', value='1'),
        SetEnvironmentVariable(name='PX4_SITL_WORLD', value=world_file),

        px4_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=px4_process,
                on_exit=[
                    LogInfo(msg='PX4 SITL process exited. Shutting down launch.'),
                    EmitEvent(event=Shutdown(reason='PX4 SITL exited')),
                ]
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=EnvironmentVariable('PX4_AUTOPILOT_PATH', default_value=''),
            description='Path to PX4-Autopilot root directory.'
        ),
        OpaqueFunction(function=_launch_setup),
    ])