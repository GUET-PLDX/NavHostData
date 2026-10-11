#include <cstddef>
#include <iostream>
#include <string_view>
#include <type_traits>

#include "HostDataLink.hpp"

using namespace Pldx::HostDataLink;

static_assert(std::string_view(CHASSIS_TARGET_TOPIC) == "nav_data");
static_assert(std::string_view(DECISION_COMMAND_TOPIC) == "behavior_data");
static_assert(std::string_view(TARGET_EULER_TOPIC) == "target_euler");
static_assert(std::string_view(FIRE_NOTIFY_TOPIC) == "fire_notify");
static_assert(std::string_view(TEAM_INFO_TOPIC) == "team_info");
static_assert(std::string_view(GIMBAL_FEEDBACK_TOPIC) == "nav_gimbal_feedback");
static_assert(sizeof(ChassisTarget) == 45U);
static_assert(sizeof(BehaviorData) == 26U);
static_assert(sizeof(TeamInfo::AllyRobotStatus) == 11U);
static_assert(sizeof(TeamInfo) == 48U);
static_assert(sizeof(GameInfo) == 22U);
static_assert(sizeof(SentryInfoOnline) == 37U);
static_assert(sizeof(SentryInfoOffline) == 16U);
static_assert(sizeof(RadarInfo) == 59U);
static_assert(sizeof(GimbalFeedback) == 28U);
static_assert(offsetof(ChassisTarget, vx_mps) == 0U);
static_assert(offsetof(ChassisTarget, use_speed_control) == 40U);
static_assert(offsetof(ChassisTarget, delta_yaw) == 41U);
static_assert(offsetof(BehaviorData, ammo_purchase_request) == 11U);
static_assert(offsetof(BehaviorData, use_gyro_mode) == 25U);
static_assert(offsetof(TeamInfo, outpost_hp) == 44U);
static_assert(offsetof(SentryInfoOffline, yaw_camerainit_to_gimbal) == 0U);
static_assert(offsetof(SentryInfoOffline, lifter_current_pos) == 4U);
static_assert(offsetof(SentryInfoOffline, is_transformable) == 5U);
static_assert(offsetof(SentryInfoOffline, transform_state) == 6U);
static_assert(offsetof(SentryInfoOffline, capacitor_capacity) == 10U);
static_assert(offsetof(SentryInfoOffline, chassis_imu_yaw) == 11U);
static_assert(offsetof(SentryInfoOffline, tunnel_yaw_aligned) == 15U);
static_assert(sizeof(Pldx::HostDataLinkDetail::HostGimbalTarget) == 36U);
static_assert(sizeof(Pldx::HostDataLinkDetail::HostFireNotify) == 1U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, rol) == 0U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, pit) == 4U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, yaw) == 8U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, rol_dot) ==
              12U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, pit_dot) ==
              16U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, yaw_dot) ==
              20U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, rol_ddot) ==
              24U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, pit_ddot) ==
              28U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostGimbalTarget, yaw_ddot) ==
              32U);
static_assert(offsetof(Pldx::HostDataLinkDetail::HostFireNotify, isfire) == 0U);
static_assert(
    std::is_trivially_copyable_v<Pldx::HostDataLinkDetail::HostGimbalTarget>);
static_assert(
    std::is_trivially_copyable_v<Pldx::HostDataLinkDetail::HostFireNotify>);

int main() {
  std::cout << sizeof(ChassisTarget) << ',' << sizeof(BehaviorData) << ','
            << sizeof(TeamInfo) << '\n';
}
