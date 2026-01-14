# Copyright 2025 Alibaba Damo Academy. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Find USB port for MotorsBus.

Usage:
    $ find-motor-port
"""

import platform
import os
import subprocess
import yaml
import time
from pathlib import Path

from RynnLeRobot.scripts.lang import select_language, t


def find_available_ports():
    from serial.tools import list_ports  # Part of pyserial library

    # Use pyserial's list_ports for all platforms - it's more reliable
    ports = [port.device for port in list_ports.comports()]
    return ports


def check_user_in_dialout():
    """Check if current user is already in dialout group."""
    try:
        result = subprocess.run(['groups'], capture_output=True, text=True)
        groups = result.stdout.strip().split()
        return 'dialout' in groups
    except Exception:
        return False


def set_port_permissions(ports):
    """Set permissions for detected ports on both Linux and macOS."""
    if not ports:
        return

    print(t("setting_permissions"))

    # On Linux, check and add user to dialout group if needed
    if platform.system() == "Linux":
        current_user = os.getenv('USER') or os.getenv('USERNAME')

        if not check_user_in_dialout():
            try:
                if current_user:
                    print(t("adding_to_dialout", user=current_user))
                    subprocess.run(['sudo', 'usermod', '-a', '-G', 'dialout', current_user], check=True, timeout=15)
                    print(t("added_to_dialout", user=current_user))
                    print(t("dialout_next_login"))

            except subprocess.CalledProcessError as e:
                print(t("dialout_add_failed", error=e))
            except Exception as e:
                print(t("dialout_error", error=e))
        else:
            print(t("already_in_dialout", user=current_user))

    # Test permissions first, then set if needed
    for port in ports:
        if os.path.exists(port):
            try:
                with open(port, 'r+b'):
                    pass
                print(t("port_accessible", port=port))
                continue
            except (PermissionError, OSError):
                pass

            try:
                print(t("setting_port_perms", port=port))
                subprocess.run(['sudo', 'chmod', '666', port], check=True, timeout=10)
                print(t("perms_set", port=port))
            except subprocess.TimeoutExpired:
                print(t("perms_timeout", port=port))
            except subprocess.CalledProcessError as e:
                print(t("perms_failed", port=port, error=e))
                print(t("perms_hint"))
                print(t("perms_hint_cmd"))
            except Exception as e:
                print(t("perms_failed", port=port, error=e))
        else:
            print(t("port_not_found", port=port))


def update_config_with_ports(detected_ports, port_type="follower", is_second_leader=False):
    """Update config.yaml with the detected ports."""
    import RynnLeRobot
    package_dir = Path(RynnLeRobot.__file__).parent
    project_root = package_dir.parent
    config_path = project_root / "configs" / "so101.yaml"

    try:
        if config_path.exists():
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
        else:
            config = {}

        if len(detected_ports) == 1:
            # Single port detected
            port = detected_ports[0]
            if port_type == "follower":
                if "follower" not in config or not isinstance(config["follower"], dict):
                    config["follower"] = {}
                config["follower"]["port"] = port
                print(t("updated_follower_port", config_path=config_path, port=port))
            elif port_type == "leader":
                if "leader" not in config or not isinstance(config["leader"], dict):
                    config["leader"] = {}
                if is_second_leader:
                    config["leader"]["port_2"] = port
                    print(t("updated_second_leader_port", config_path=config_path, port=port))
                else:
                    config["leader"]["port"] = port
                    print(t("updated_leader_port", config_path=config_path, port=port))

        with open(config_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    except Exception as e:
        print(t("config_update_failed", error=e))
        print(t("manual_set_ports", config_path=config_path))


def find_port(dev_type, is_second_leader=False):
    # Map display name to config key
    if "follower" in dev_type.lower():
        config_type = "follower"
    elif "leader" in dev_type.lower():
        config_type = "leader"
    else:
        config_type = dev_type

    # Set default based on whether it's the second leader
    default = "y/N" if is_second_leader else "Y/n"
    default_val = "n" if is_second_leader else "y"

    response = input(t("detect_port_prompt", dev_type=dev_type, default=default, default_val=default_val.upper())).strip().lower()

    # If empty response, use default
    if not response:
        response = default_val

    if response in ['n', 'no']:
        print(t("skip_detection", dev_type=dev_type))
        return None

    print(t("usb_remove_prompt"))
    input()
    ports_before = find_available_ports()

    time.sleep(0.5)

    print(t("usb_connect_prompt"))
    input()

    print(t("waiting_device"))
    time.sleep(2)

    ports_after = find_available_ports()
    ports_diff = list(set(ports_after) - set(ports_before))

    if len(ports_diff) == 1:
        port = ports_diff[0]
        print(t("detected_port", dev_type=dev_type, port=port))
        print(t("detected_port_env", port=port))
        update_config_with_ports(ports_diff, config_type, is_second_leader)
        set_port_permissions(ports_diff)
        print()
        return port
    elif len(ports_diff) == 0:
        print(t("no_port_diff", dev_type=dev_type))
        return None
    else:
        print(t("too_many_ports", dev_type=dev_type, count=len(ports_diff), ports=ports_diff))
        return None


def main():
    """Main entry point for the find-motor-port command."""
    # Select language if not set via environment
    select_language()

    follower_port = find_port(t("dev_type_follower"))

    leader_port = find_port(t("dev_type_leader"))

    second_leader_port = find_port(t("dev_type_second_leader"), is_second_leader=True)

    print(t("port_summary"))
    if follower_port:
        print(t("follower_port", port=follower_port))
    else:
        print(t("follower_port", port=t("not_detected")))

    if leader_port:
        print(t("leader_port", port=leader_port))
    else:
        print(t("leader_port", port=t("not_detected")))

    if second_leader_port:
        print(t("second_leader_port", port=second_leader_port))
    else:
        print(t("second_leader_port", port=t("not_detected")))
    print("=" * 60)

    # Force clean exit to avoid memory corruption during cleanup
    os._exit(0)


if __name__ == "__main__":
    main()
