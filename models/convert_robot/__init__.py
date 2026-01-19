"""
Robot Package to RynnMotion MJCF Converter

Self-contained tool to convert ROS URDF packages or MuJoCo Menagerie robots
to RynnMotion's modular MJCF layout.

Usage:
    cd models/convert_robot
    python convert_robot.py --urdf /path/to/package --output ../3.robot_arm/XX.robot
    python convert_robot.py --menagerie /path/to/menagerie/robot --output ../3.robot_arm/XX.robot
"""

__version__ = "0.1.0"
