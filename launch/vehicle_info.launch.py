from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paket dizinini al
    package_share_directory = get_package_share_directory('control_planning_ros')

    # YAML config dosyasının yolunu al
    yaml_config_path = os.path.join(
        package_share_directory,
        'config',
        'vehicle_dimension.yaml'
    )

    # YAML dosyasının mevcut olup olmadığını kontrol et
    if not os.path.exists(yaml_config_path):
        raise FileNotFoundError(f"YAML config file not found: {yaml_config_path}")

    # Vehicle Info düğümünü başlat
    vehicle_info_node = Node(
        package='control_planning_ros',
        executable='vehicle_info',
        name='control_node',
        parameters=[yaml_config_path],
        output='screen'  # Logları terminalde görmek için
    )

    return LaunchDescription([vehicle_info_node])
