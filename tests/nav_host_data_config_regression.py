#!/usr/bin/env python3

import re
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
GIMBAL_CONFIG = ROOT / "User/RobotConfig/sentry_gimbal.yaml"
MODULES_CONFIG = ROOT / "Modules/modules.yaml"
NAV_HOST_DATA_HEADER = ROOT / "Modules/NavHostData/NavHostData.hpp"


def module_by_id(modules, module_id):
    matches = [module for module in modules if module.get("id") == module_id]
    assert len(matches) == 1, f"expected one module id {module_id}, got {len(matches)}"
    return matches[0]


def main():
    config = yaml.safe_load(GIMBAL_CONFIG.read_text(encoding="utf-8"))
    modules = config["modules"]

    assert all(module.get("name") != "WsProtocol" for module in modules)

    allowed_uart_owners = {"NavHostData", "SharedTopic", "SharedTopicClient"}
    for module in modules:
        args = module.get("constructor_args", {})
        if "usb_otg_hs_navigation_cdc" in str(args):
            assert module["name"] in allowed_uart_owners, module

    nav_link = module_by_id(modules, "nav_host_data")
    assert nav_link["name"] == "NavHostData"
    assert nav_link["constructor_args"]["cmd"] == "@cmd"
    assert nav_link["constructor_args"]["uart_name"] == "usb_otg_hs_navigation_cdc"

    nav_rx = module_by_id(modules, "nav_rx_shared_topic")
    assert nav_rx["name"] == "SharedTopic"
    assert nav_rx["constructor_args"]["uart_name"] == "usb_otg_hs_navigation_cdc"
    assert nav_rx["constructor_args"]["topic_configs"] == [
        "nav_data",
        "behavior_data",
        "target_euler",
        "fire_notify",
    ]

    nav_tx = module_by_id(modules, "nav_tx_shared_topic_client")
    assert nav_tx["name"] == "SharedTopicClient"
    assert nav_tx["constructor_args"]["uart_name"] == "usb_otg_hs_navigation_cdc"
    assert nav_tx["constructor_args"]["slot_count"] >= 32
    assert nav_tx["constructor_args"]["topic_configs"] == [
        "team_info",
        "game_info",
        "online_info",
        "offline_info",
        "radar_info",
    ]

    # 视觉共享主题模块已移除；视觉目标/开火改由 NavHostData 命名主题直接订阅。
    assert all(module.get("id") != "vision_shared_topic" for module in modules)
    assert all(
        module.get("id") != "vision_shared_topic_client" for module in modules
    )

    local_status_topic = "host_chassis_session_status"
    for module in (nav_rx, nav_tx):
        assert local_status_topic not in module["constructor_args"]["topic_configs"]

    assert all(module.get("name") != "HostData" for module in modules)

    registry = yaml.safe_load(MODULES_CONFIG.read_text(encoding="utf-8"))["modules"]
    assert "pldx/WsProtocol" not in registry
    assert "pldx/NavHostData" in registry
    assert "pldx/NavLinkProtocol" not in registry
    assert "pldx/NavHostLink" not in registry

    source = NAV_HOST_DATA_HEADER.read_text(encoding="utf-8")
    assert "ChassisTargetV1" not in source
    assert "DecisionCommandV1" not in source
    assert "FeedAI" in source
    assert '"nav_chassis_mode"' in source
    assert '"use_capacitor"' in source
    assert '"yawmotor_omega"' in source
    assert '"pitchmotor_omega"' in source
    assert "FEEDBACK_ANGULAR_VELOCITY_VALID" in source
    assert "CMD::Mode::CMD_AUTO_CTRL" in source
    assert "NAVIGATION_SPEED_LIMIT" not in source
    assert "NormalizeLinearVelocity(" not in source
    assert "NormalizeAngularVelocity(" not in source
    assert "navigation_velocity.vx_mps = latest_chassis_.vx_mps" in source
    assert "navigation_velocity.vy_mps = latest_chassis_.vy_mps" in source
    assert "navigation_velocity.wz_rad_s = latest_chassis_.vw_rad_s" in source
    assert "latest_chassis_.vx_mps" in source
    assert "latest_chassis_.vy_mps" in source
    assert "latest_chassis_.vw_rad_s" in source
    assert "if (tracker_.Observe(value, NOW))" in source
    assert source.index("tracker_.Observe(value, NOW)") < source.index(
        "latest_chassis_ = value"
    )
    print("PASS: native navigation and vision ownership configuration")


if __name__ == "__main__":
    main()
