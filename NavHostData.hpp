#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ROS2_LIBXR navigation link protocol and host endpoint
constructor_args:
  - cmd: '@cmd'
  - uart_name: "usb_otg_hs_navigation_cdc"
  - referee_topic_name: "sentry_ref"
  - pitch_topic_name: "pitchmotor_angle"
  - yaw_topic_name: "yawmotor_angle"
  - quaternion_topic_name: "ahrs_quaternion"
required_hardware: uart_name
depends:
  - pldx/CMD
  - pldx/Referee
=== END MANIFEST === */
// clang-format on

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"

namespace Pldx::NavHostData {

inline constexpr char CHASSIS_TARGET_TOPIC[] = "nav_data";
inline constexpr char DECISION_COMMAND_TOPIC[] = "behavior_data";
inline constexpr char TARGET_EULER_TOPIC[] = "target_euler";
inline constexpr char FIRE_NOTIFY_TOPIC[] = "fire_notify";
inline constexpr char TEAM_INFO_TOPIC[] = "team_info";
inline constexpr char GAME_INFO_TOPIC[] = "game_info";
inline constexpr char ONLINE_INFO_TOPIC[] = "online_info";
inline constexpr char OFFLINE_INFO_TOPIC[] = "offline_info";
inline constexpr char RADAR_INFO_TOPIC[] = "radar_info";
inline constexpr char GIMBAL_FEEDBACK_TOPIC[] = "nav_gimbal_feedback";
inline constexpr char LAUNCHER_FEEDBACK_TOPIC[] = "nav_launcher_feedback_v1";

inline constexpr uint8_t FEEDBACK_YAW_VALID = 1U << 0U;
inline constexpr uint8_t FEEDBACK_PITCH_VALID = 1U << 1U;
inline constexpr uint8_t FEEDBACK_ANGULAR_VELOCITY_VALID = 1U << 2U;
inline constexpr uint8_t FEEDBACK_BULLET_SPEED_VALID = 1U << 3U;
inline constexpr uint8_t FEEDBACK_BULLET_COUNT_VALID = 1U << 4U;
inline constexpr uint8_t FEEDBACK_GIMBAL_MODE_VALID = 1U << 5U;
inline constexpr uint8_t FEEDBACK_VISION_TASK_VALID = 1U << 6U;
inline constexpr uint8_t FEEDBACK_VALID_MASK = 0x7FU;
inline constexpr uint8_t GIMBAL_BULLET_SPEED_VALID =
    FEEDBACK_BULLET_SPEED_VALID;
inline constexpr uint8_t GIMBAL_BULLET_COUNT_VALID =
    FEEDBACK_BULLET_COUNT_VALID;
inline constexpr uint8_t GIMBAL_MODE_VALID = FEEDBACK_GIMBAL_MODE_VALID;
inline constexpr uint8_t GIMBAL_SHOOT_MODE_VALID = FEEDBACK_VISION_TASK_VALID;

// Compact launcher sample consumed by NavHostData before publishing the
// aggregate 28-byte GimbalFeedback payload.
struct [[gnu::packed]] GimbalFeedbackV1 {
  float bullet_speed_mps{};
  uint16_t bullet_count{};
  uint8_t gimbal_mode{};
  uint8_t shoot_mode{};
  uint8_t valid_flags{};
  uint8_t reserved[3]{};
};

inline constexpr uint16_t REF_CMD_GAME_STATUS = 0x0001U;
inline constexpr uint16_t REF_CMD_GAME_ROBOT_HP = 0x0003U;
inline constexpr uint16_t REF_CMD_FIELD_EVENTS = 0x0101U;
inline constexpr uint16_t REF_CMD_ROBOT_STATUS = 0x0201U;
inline constexpr uint16_t REF_CMD_POWER_HEAT = 0x0202U;
inline constexpr uint16_t REF_CMD_ROBOT_POSITION = 0x0203U;
inline constexpr uint16_t REF_CMD_ROBOT_BUFF = 0x0204U;
inline constexpr uint16_t REF_CMD_ROBOT_DAMAGE = 0x0206U;
inline constexpr uint16_t REF_CMD_BULLET_REMAINING = 0x0208U;
inline constexpr uint16_t REF_CMD_RFID = 0x0209U;
inline constexpr uint16_t REF_CMD_GROUND_ROBOT_POSITION = 0x020BU;
inline constexpr uint16_t REF_CMD_SENTRY_INFO = 0x020DU;

inline constexpr uint16_t SOURCE_GAME_STATUS = 1U << 0U;
inline constexpr uint16_t SOURCE_ROBOT_HP = 1U << 1U;
inline constexpr uint16_t SOURCE_FIELD_EVENT = 1U << 2U;
inline constexpr uint16_t SOURCE_ROBOT_STATUS = 1U << 3U;
inline constexpr uint16_t SOURCE_POWER_HEAT = 1U << 4U;
inline constexpr uint16_t SOURCE_ROBOT_POSITION = 1U << 5U;
inline constexpr uint16_t SOURCE_ROBOT_BUFF = 1U << 6U;
inline constexpr uint16_t SOURCE_ROBOT_DAMAGE = 1U << 7U;
inline constexpr uint16_t SOURCE_BULLET_REMAINING = 1U << 8U;
inline constexpr uint16_t SOURCE_RFID = 1U << 9U;
inline constexpr uint16_t SOURCE_SENTRY_POS = 1U << 10U;
inline constexpr uint16_t SUPPORTED_REFEREE_SOURCE_MASK =
    SOURCE_GAME_STATUS | SOURCE_ROBOT_HP | SOURCE_FIELD_EVENT |
    SOURCE_ROBOT_STATUS | SOURCE_POWER_HEAT | SOURCE_ROBOT_POSITION |
    SOURCE_ROBOT_BUFF | SOURCE_ROBOT_DAMAGE | SOURCE_BULLET_REMAINING |
    SOURCE_RFID | SOURCE_SENTRY_POS;
inline constexpr uint16_t REQUIRED_REFEREE_SOURCE_MASK =
    SOURCE_GAME_STATUS | SOURCE_ROBOT_HP | SOURCE_FIELD_EVENT |
    SOURCE_ROBOT_STATUS | SOURCE_POWER_HEAT | SOURCE_ROBOT_BUFF |
    SOURCE_BULLET_REMAINING | SOURCE_RFID | SOURCE_SENTRY_POS;

// ROS2_LIBXR wire payloads. Field order and one-byte packing are part of the
// external contract; do not reorder or substitute integer widths.
struct [[gnu::packed]] ChassisTarget {
  float vx_mps{};           /* target velocity, m/s */
  float vy_mps{};           /* target velocity, m/s */
  float vw_rad_s{};         /* target yaw rate, rad/s */
  float current_yaw{};      /* navigation yaw feedback, rad */
  float current_vx{};       /* measured velocity, m/s */
  float current_vy{};       /* measured velocity, m/s */
  float current_vw{};       /* measured yaw rate, rad/s */
  float fx_global{};        /* global-frame force, N */
  float fy_global{};        /* global-frame force, N */
  float fw_global{};        /* global-frame yaw torque, N*m */
  bool use_speed_control{}; /* true: speed path, false: direct force */
  float delta_yaw{};        /* navigation/IMU yaw delta, rad */
};

struct [[gnu::packed]] BehaviorData {
  uint8_t pitch_mode{};
  uint8_t desire_stance{};
  uint8_t desire_lifter_pos{};
  float scan_yaw_min_rad{}; /* rad */
  float scan_yaw_max_rad{}; /* rad */
  uint16_t ammo_purchase_request{};
  uint8_t revive_request{};
  uint8_t remote_revive_request{};
  uint8_t remote_ammo_request{};
  uint8_t remote_health_request{};
  bool use_limited_scan{};
  bool not_aim_enemy{};
  bool use_capacitor{};
  bool tunnel_align_active{};
  float tunnel_align_angle_rad{}; /* rad */
  bool use_gyro_mode{};
};

struct [[gnu::packed]] TeamInfo {
  struct [[gnu::packed]] AllyRobotStatus {
    uint8_t robot_id{};
    uint16_t robot_hp{};
    float robot_pos_x{};
    float robot_pos_y{};
  };
  AllyRobotStatus ally_status[4]{};
  uint16_t outpost_hp{};
  uint16_t base_hp{};
};

struct [[gnu::packed]] GameInfo {
  uint16_t game_time_remaining{};
  uint16_t coin_remaining{};
  uint32_t event_code{};
  uint8_t game_status{};
  float manual_point_x{};
  float manual_point_y{};
  uint8_t manual_key{};
  uint16_t enemy_outpost_hp{};
  uint16_t enemy_base_hp{};
};

struct [[gnu::packed]] SentryInfoOnline {
  uint16_t self_health{};
  uint16_t bullets_remaining{};
  uint16_t cooling_value{};
  uint16_t heat_limit{};
  uint16_t current_heat{};
  float sentry_pos_x{};
  float sentry_pos_y{};
  float speed_monitor_angle{};
  uint32_t sentry_info_1{};
  uint16_t sentry_info_2{};
  uint64_t sentry_info_3{};
  uint8_t energy_ratio{};
};

struct [[gnu::packed]] SentryInfoOffline {
  float yaw_camerainit_to_gimbal{}; /* deg, counterclockwise positive */
  uint8_t lifter_current_pos{};
  bool is_transformable{};
  float transform_state{};
  uint8_t capacitor_capacity{}; /* 0..100 percent; 255 means unavailable */
  float chassis_imu_yaw{};      /* rad */
  // True when the active tunnel-alignment request is within the firmware
  // alignment tolerance.
  bool tunnel_yaw_aligned{};
};

struct [[gnu::packed]] EnemyRobotStatus {
  uint8_t robot_id{};
  uint16_t robot_hp{};
  uint16_t allowed_projectile{};
  uint16_t robot_pos_x{};
  uint16_t robot_pos_y{};
};

struct [[gnu::packed]] RadarInfo {
  EnemyRobotStatus enemy_status[6]{};
  uint16_t enemy_coin_left{};
  uint16_t enemy_coin_accumulated{};
  bool is_enemy_outpost_sensed{};
};

struct [[gnu::packed]] GimbalFeedback {
  float yaw{};
  float pitch{};
  float yaw_velocity{};
  float pitch_velocity{};
  float bullet_speed{};
  uint16_t bullet_count{};
  uint8_t gimbal_mode{};
  uint8_t vision_task{};
  uint8_t valid_flags{};
  uint8_t reserved[3]{};
};

static_assert(std::endian::native == std::endian::little);
static_assert(sizeof(ChassisTarget) == 45U);
static_assert(sizeof(BehaviorData) == 26U);
static_assert(sizeof(TeamInfo::AllyRobotStatus) == 11U);
static_assert(sizeof(TeamInfo) == 48U);
static_assert(sizeof(GameInfo) == 22U);
static_assert(sizeof(SentryInfoOnline) == 37U);
static_assert(sizeof(SentryInfoOffline) == 16U);
static_assert(sizeof(EnemyRobotStatus) == 9U);
static_assert(sizeof(RadarInfo) == 59U);
static_assert(sizeof(GimbalFeedback) == 28U);
static_assert(sizeof(GimbalFeedbackV1) == 12U);
static_assert(offsetof(ChassisTarget, use_speed_control) == 40U);
static_assert(offsetof(ChassisTarget, delta_yaw) == 41U);
static_assert(offsetof(BehaviorData, ammo_purchase_request) == 11U);
static_assert(offsetof(BehaviorData, use_gyro_mode) == 25U);
static_assert(offsetof(SentryInfoOffline, yaw_camerainit_to_gimbal) == 0U);
static_assert(offsetof(SentryInfoOffline, lifter_current_pos) == 4U);
static_assert(offsetof(SentryInfoOffline, is_transformable) == 5U);
static_assert(offsetof(SentryInfoOffline, transform_state) == 6U);
static_assert(offsetof(SentryInfoOffline, capacitor_capacity) == 10U);
static_assert(offsetof(SentryInfoOffline, chassis_imu_yaw) == 11U);
static_assert(offsetof(SentryInfoOffline, tunnel_yaw_aligned) == 15U);
static_assert(offsetof(TeamInfo, outpost_hp) == 44U);
static_assert(std::is_trivially_copyable_v<ChassisTarget>);
static_assert(std::is_trivially_copyable_v<BehaviorData>);
static_assert(std::is_trivially_copyable_v<TeamInfo>);
static_assert(std::is_trivially_copyable_v<GameInfo>);
static_assert(std::is_trivially_copyable_v<SentryInfoOnline>);
static_assert(std::is_trivially_copyable_v<SentryInfoOffline>);
static_assert(std::is_trivially_copyable_v<RadarInfo>);

}  // namespace Pldx::NavHostData

namespace Pldx::HostChassisSession {

inline constexpr char STATUS_TOPIC[] = "host_chassis_session_status";
inline constexpr uint32_t FRESHNESS_TIMEOUT_MS = 150U;

struct Status {
  uint32_t accepted_sequence = 0U;
  uint32_t accepted_time_ms = 0U;
  bool accepted_seen = false;
  bool armed_fresh = false;
};

inline bool IsFresh(const Status& status, LibXR::MillisecondTimestamp now) {
  return status.accepted_seen && status.armed_fresh &&
         (now - LibXR::MillisecondTimestamp(status.accepted_time_ms))
                 .ToMillisecond() <= FRESHNESS_TIMEOUT_MS;
}

}  // namespace Pldx::HostChassisSession

namespace Pldx::NavHostDataDetail {

inline constexpr uint32_t COMMAND_TIMEOUT_MS = 150U;
inline constexpr uint32_t VISUAL_SOURCE_TIMEOUT_MS = 250U;
inline constexpr float CHASSIS_YAW_ALIGNMENT_TOLERANCE_RAD = 0.05F;
inline constexpr uint32_t AUTO_CHASSIS_MODE_ROTOR = 2U;
inline constexpr uint32_t AUTO_CHASSIS_MODE_NAVIGATION = 4U;
inline constexpr uint32_t FEEDBACK_OMEGA_TIMEOUT_MS = 50U;

inline bool AngularVelocityFresh(LibXR::MillisecondTimestamp now,
                                 LibXR::MillisecondTimestamp yaw_time,
                                 LibXR::MillisecondTimestamp pitch_time,
                                 bool yaw_seen, bool pitch_seen) {
  return yaw_seen && pitch_seen &&
         (now - yaw_time).ToMillisecond() <= FEEDBACK_OMEGA_TIMEOUT_MS &&
         (now - pitch_time).ToMillisecond() <= FEEDBACK_OMEGA_TIMEOUT_MS;
}

inline float YawRadToCamerainitDeg(float yaw_rad) {
  if (!std::isfinite(yaw_rad)) {
    return 0.0F;
  }
  const float DEG = yaw_rad * (180.0F / static_cast<float>(LibXR::PI));
  float wrapped = std::remainder(DEG, 360.0F);
  if (wrapped <= -180.0F) {
    wrapped = 180.0F;
  }
  return wrapped;
}

struct HostGimbalTarget {
  float rol{}, pit{}, yaw{};
  float rol_dot{}, pit_dot{}, yaw_dot{};
  float rol_ddot{}, pit_ddot{}, yaw_ddot{};
};
struct HostFireNotify {
  bool isfire{};
};

static_assert(sizeof(HostGimbalTarget) == 36U);
static_assert(sizeof(HostFireNotify) == 1U);
static_assert(offsetof(HostGimbalTarget, rol) == 0U);
static_assert(offsetof(HostGimbalTarget, pit) == 4U);
static_assert(offsetof(HostGimbalTarget, yaw) == 8U);
static_assert(offsetof(HostGimbalTarget, rol_dot) == 12U);
static_assert(offsetof(HostGimbalTarget, pit_dot) == 16U);
static_assert(offsetof(HostGimbalTarget, yaw_dot) == 20U);
static_assert(offsetof(HostGimbalTarget, rol_ddot) == 24U);
static_assert(offsetof(HostGimbalTarget, pit_ddot) == 28U);
static_assert(offsetof(HostGimbalTarget, yaw_ddot) == 32U);
static_assert(offsetof(HostFireNotify, isfire) == 0U);
static_assert(std::is_trivially_copyable_v<HostGimbalTarget>);
static_assert(std::is_trivially_copyable_v<HostFireNotify>);

inline bool IsTunnelYawAligned(float chassis_yaw_rad, bool align_active,
                               float target_yaw_rad, bool yaw_valid) {
  if (!align_active || !yaw_valid || !std::isfinite(chassis_yaw_rad) ||
      !std::isfinite(target_yaw_rad)) {
    return false;
  }
  const float error = std::remainder(chassis_yaw_rad - target_yaw_rad,
                                     static_cast<float>(LibXR::TWO_PI));
  return std::fabs(error) <= CHASSIS_YAW_ALIGNMENT_TOLERANCE_RAD;
}

inline bool ChassisCommandValid(const NavHostData::ChassisTarget& command) {
  return std::isfinite(command.vx_mps) && std::isfinite(command.vy_mps) &&
         std::isfinite(command.vw_rad_s) &&
         std::isfinite(command.current_yaw) &&
         std::isfinite(command.current_vx) &&
         std::isfinite(command.current_vy) &&
         std::isfinite(command.current_vw) && std::isfinite(command.delta_yaw);
}

class ChassisCommandTracker {
 public:
  bool Observe(const NavHostData::ChassisTarget& command,
               LibXR::MillisecondTimestamp now) {
    Expire(now);
    command_seen_ = true;
    schema_ok_ = true;
    finite_ = ChassisCommandValid(command);
    if (!finite_) return Reject();
    Accept(now);
    return true;
  }

  bool CommandFresh(LibXR::MillisecondTimestamp now) {
    Expire(now);
    return armed_ && accepted_command_seen_;
  }
  uint32_t StatusFlags(LibXR::MillisecondTimestamp now) {
    uint32_t flags = 1U;
    if (command_seen_) flags |= 1U << 1U;
    if (CommandFresh(now)) flags |= 1U << 2U;
    if (schema_ok_) flags |= 1U << 3U;
    if (finite_) flags |= 1U << 4U;
    return flags;
  }
  uint32_t CommandAge(LibXR::MillisecondTimestamp now) const {
    return accepted_command_seen_ ? (now - last_command_time_).ToMillisecond()
                                  : std::numeric_limits<uint32_t>::max();
  }
  uint32_t LastSequence() const { return accepted_count_; }
  uint32_t RejectedCount() const { return rejected_count_; }

 private:
  void Accept(LibXR::MillisecondTimestamp now) {
    last_command_time_ = now;
    accepted_command_seen_ = true;
    armed_ = true;
    ++accepted_count_;
  }
  bool Reject() {
    ++rejected_count_;
    return false;
  }
  void Expire(LibXR::MillisecondTimestamp now) {
    if (armed_ &&
        (now - last_command_time_).ToMillisecond() >= COMMAND_TIMEOUT_MS) {
      armed_ = false;
    }
  }
  LibXR::MillisecondTimestamp last_command_time_{};
  uint32_t rejected_count_{}, accepted_count_{};
  bool command_seen_{}, accepted_command_seen_{}, schema_ok_{}, finite_{};
  bool armed_{};
};

struct DecisionPublication {
  bool accepted = false;
  bool publish_buy_bullet_num = false;
  bool publish_remote_buy_bullet_times = false;
  bool publish_remote_buy_hp_times = false;
  bool publish_buy_resurrection = false;
  uint16_t buy_bullet_num = 0U;
  uint8_t remote_buy_bullet_times = 0U;
  uint8_t remote_buy_hp_times = 0U;
  bool buy_resurrection = false;
};

class DecisionSequenceTracker {
 public:
  DecisionPublication Decode(const NavHostData::BehaviorData& value,
                             bool ready = true) {
    DecisionPublication result;
    if (!ready || !std::isfinite(value.scan_yaw_min_rad) ||
        !std::isfinite(value.scan_yaw_max_rad) ||
        !std::isfinite(value.tunnel_align_angle_rad)) {
      ++rejected_count_;
      return result;
    }
    result.accepted = true;
    if (!seen_) {
      seen_ = true;
    } else {
      result.buy_bullet_num = Delta(value.ammo_purchase_request, ammo_);
      result.remote_buy_bullet_times =
          Delta(value.remote_ammo_request, remote_ammo_);
      result.remote_buy_hp_times =
          Delta(value.remote_health_request, remote_hp_);
      const bool rollback = value.ammo_purchase_request < ammo_ ||
                            value.remote_ammo_request < remote_ammo_ ||
                            value.remote_health_request < remote_hp_ ||
                            value.revive_request < revive_ ||
                            value.remote_revive_request < remote_revive_;
      if (!rollback) {
        result.publish_buy_bullet_num = result.buy_bullet_num != 0U;
        result.publish_remote_buy_bullet_times =
            result.remote_buy_bullet_times != 0U;
        result.publish_remote_buy_hp_times = result.remote_buy_hp_times != 0U;
        result.publish_buy_resurrection =
            value.revive_request != revive_ ||
            value.remote_revive_request != remote_revive_;
        result.buy_resurrection = result.publish_buy_resurrection;
      }
    }
    ammo_ = value.ammo_purchase_request;
    remote_ammo_ = value.remote_ammo_request;
    remote_hp_ = value.remote_health_request;
    revive_ = value.revive_request;
    remote_revive_ = value.remote_revive_request;
    return result;
  }
  uint32_t RejectedCount() const { return rejected_count_; }

 private:
  template <typename T>
  static T Delta(T value, T previous) {
    return value >= previous ? static_cast<T>(value - previous) : T{};
  }
  uint16_t ammo_{};
  uint8_t remote_ammo_{}, remote_hp_{}, revive_{}, remote_revive_{};
  uint32_t rejected_count_{};
  bool seen_{};
};

template <typename Sink>
bool DispatchDecision(const DecisionPublication& value, Sink& sink) {
  if (!value.accepted) return false;
  if (value.publish_buy_bullet_num)
    sink.PublishBuyBulletNum(value.buy_bullet_num);
  if (value.publish_remote_buy_bullet_times)
    sink.PublishRemoteBuyBulletTimes(value.remote_buy_bullet_times);
  if (value.publish_remote_buy_hp_times)
    sink.PublishRemoteBuyHpTimes(value.remote_buy_hp_times);
  if (value.publish_buy_resurrection)
    sink.PublishBuyResurrection(value.buy_resurrection);
  return true;
}

}  // namespace Pldx::NavHostDataDetail

#if __has_include("app_framework.hpp")
#include "CMD.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "timebase.hpp"
#include "uart.hpp"

class NavHostData : public LibXR::Application {
 public:
  NavHostData(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
              CMD& cmd, const char* uart_name, const char* referee_topic_name,
              const char* pitch_topic_name, const char* yaw_topic_name,
              const char* quaternion_topic_name)
      : cmd_(&cmd),
        uart_(hw.template FindOrExit<LibXR::UART>({uart_name})),
        chassis_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavHostData::ChassisTarget>(
                "nav_data")),
        behavior_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavHostData::BehaviorData>(
                "behavior_data")),
        team_info_topic_(LibXR::Topic::CreateTopic<Pldx::NavHostData::TeamInfo>(
            "team_info")),
        game_info_topic_(LibXR::Topic::CreateTopic<Pldx::NavHostData::GameInfo>(
            "game_info")),
        online_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavHostData::SentryInfoOnline>(
                "online_info")),
        offline_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavHostData::SentryInfoOffline>(
                "offline_info")),
        radar_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavHostData::RadarInfo>(
                "radar_info")),
        gimbal_feedback_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavHostData::GimbalFeedback>(
                Pldx::NavHostData::GIMBAL_FEEDBACK_TOPIC)),
        nav_chassis_mode_topic_(LibXR::Topic::CreateTopic<uint32_t>(
            "nav_chassis_mode", nullptr, false)),
        use_capacitor_topic_(
            LibXR::Topic::CreateTopic<bool>("use_capacitor", nullptr, false)) {
    ASSERT(uart_->SetConfig({921600U, LibXR::UART::Parity::NO_PARITY, 8U,
                             1U}) == LibXR::ErrorCode::OK);
    RegisterRawCallback<&NavHostData::OnChassis>(chassis_topic_);
    RegisterRawCallback<&NavHostData::OnBehavior>(behavior_topic_);
    RegisterNamedCallback<Referee::RobotGameRefereePack,
                          &NavHostData::OnReferee>(referee_topic_name);
    RegisterNamedCallback<Pldx::NavHostDataDetail::HostGimbalTarget,
                          &NavHostData::OnVisualGimbal>(
        Pldx::NavHostData::TARGET_EULER_TOPIC);
    RegisterNamedCallback<Pldx::NavHostDataDetail::HostFireNotify,
                          &NavHostData::OnVisualFire>(
        Pldx::NavHostData::FIRE_NOTIFY_TOPIC);
    RegisterNamedCallback<Pldx::NavHostData::GimbalFeedbackV1,
                          &NavHostData::OnLauncherFeedback>(
        Pldx::NavHostData::LAUNCHER_FEEDBACK_TOPIC);
    RegisterNamedCallback<float, &NavHostData::OnGimbalYaw>(yaw_topic_name);
    RegisterNamedCallback<float, &NavHostData::OnGimbalPitch>(pitch_topic_name);
    RegisterNamedCallback<float, &NavHostData::OnGimbalYawOmega>(
        "yawmotor_omega");
    RegisterNamedCallback<float, &NavHostData::OnGimbalPitchOmega>(
        "pitchmotor_omega");
    RegisterNamedCallback<uint8_t, &NavHostData::OnGimbalMode>("gimbal_mode");
    RegisterNamedCallback<uint8_t, &NavHostData::OnVisionTask>("vision_task");
    RegisterNamedCallback<float, &NavHostData::OnChassisYaw>("chassis_imu_yaw");
    RegisterNamedCallback<bool, &NavHostData::OnChassisYawValid>(
        "chassis_imu_yaw_valid");
    RegisterNamedCallback<uint8_t, &NavHostData::OnCapacitorCapacity>(
        "chassis_capacitor_capacity");
    RegisterNamedCallback<bool, &NavHostData::OnCapacitorValid>(
        "chassis_capacitor_valid");
    UNUSED(pitch_topic_name);
    UNUSED(yaw_topic_name);
    UNUSED(quaternion_topic_name);
    app.Register(*this);
  }
  void OnMonitor() override {}

 private:
  static LibXR::MillisecondTimestamp Now() {
    return LibXR::Timebase::GetMilliseconds();
  }
  template <typename Data, void (NavHostData::*Method)(const Data&)>
  void RegisterCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool, NavHostData* self, const Data& data) {
          (self->*Method)(data);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <
      void (NavHostData::*Method)(const Pldx::NavHostData::ChassisTarget&)>
  void RegisterRawCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool, NavHostData* self,
           const LibXR::Topic::RawMessageView& message) {
          if (message.timestamp == 0U || message.payload.addr_ == nullptr ||
              message.payload.size_ !=
                  sizeof(Pldx::NavHostData::ChassisTarget)) {
            return;
          }
          Pldx::NavHostData::ChassisTarget value{};
          LibXR::Memory::FastCopy(&value, message.payload.addr_, sizeof(value));
          (self->*Method)(value);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <void (NavHostData::*Method)(const Pldx::NavHostData::BehaviorData&)>
  void RegisterRawCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool, NavHostData* self,
           const LibXR::Topic::RawMessageView& message) {
          if (message.timestamp == 0U || message.payload.addr_ == nullptr ||
              message.payload.size_ !=
                  sizeof(Pldx::NavHostData::BehaviorData)) {
            return;
          }
          Pldx::NavHostData::BehaviorData value{};
          LibXR::Memory::FastCopy(&value, message.payload.addr_, sizeof(value));
          (self->*Method)(value);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <typename Data, void (NavHostData::*Method)(const Data&)>
  void RegisterNamedCallback(const char* name) {
    LibXR::Topic topic(LibXR::Topic::FindOrCreate<Data>(name));
    RegisterCallback<Data, Method>(topic);
  }
  void OnChassis(const Pldx::NavHostData::ChassisTarget& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    const auto NOW = Now();
    if (tracker_.Observe(value, NOW)) {
      latest_chassis_ = value;
    }
    cmd_->FeedAI(BuildAI(NOW));
  }
  void OnBehavior(const Pldx::NavHostData::BehaviorData& value) {
    Pldx::NavHostDataDetail::DecisionPublication publication;
    bool publish_chassis_mode = false;
    uint32_t chassis_mode =
        Pldx::NavHostDataDetail::AUTO_CHASSIS_MODE_NAVIGATION;
    {
      LibXR::Mutex::LockGuard lock(mutex_);
      tunnel_align_active_ = value.tunnel_align_active;
      tunnel_align_angle_rad_ = value.tunnel_align_angle_rad;
      PublishOffline();
      publication = decision_.Decode(value);
      publish_chassis_mode = cmd_->GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL;
      chassis_mode =
          value.use_gyro_mode
              ? Pldx::NavHostDataDetail::AUTO_CHASSIS_MODE_ROTOR
              : Pldx::NavHostDataDetail::AUTO_CHASSIS_MODE_NAVIGATION;
    }
    bool use_capacitor = value.use_capacitor;
    use_capacitor_topic_.Publish(use_capacitor);
    if (publish_chassis_mode) {
      nav_chassis_mode_topic_.Publish(chassis_mode);
    }
    Sink sink(*this);
    Pldx::NavHostDataDetail::DispatchDecision(publication, sink);
  }
  void OnVisualGimbal(const Pldx::NavHostDataDetail::HostGimbalTarget& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    visual_gimbal_ = value;
    visual_gimbal_time_ = Now();
    visual_gimbal_seen_ = true;
    cmd_->FeedAI(BuildAI(visual_gimbal_time_));
  }
  void OnVisualFire(const Pldx::NavHostDataDetail::HostFireNotify& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    visual_fire_ = value;
    visual_fire_time_ = Now();
    visual_fire_seen_ = true;
    cmd_->FeedAI(BuildAI(visual_fire_time_));
  }
  void OnLauncherFeedback(const Pldx::NavHostData::GimbalFeedbackV1& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if ((value.valid_flags &
         static_cast<uint8_t>(~Pldx::NavHostData::FEEDBACK_VALID_MASK)) != 0U)
      return;
    gimbal_feedback_.bullet_speed = value.bullet_speed_mps;
    gimbal_feedback_.bullet_count = value.bullet_count;
    constexpr uint8_t kLauncherFlags =
        Pldx::NavHostData::FEEDBACK_BULLET_SPEED_VALID |
        Pldx::NavHostData::FEEDBACK_BULLET_COUNT_VALID;
    gimbal_feedback_.valid_flags =
        static_cast<uint8_t>((gimbal_feedback_.valid_flags & ~kLauncherFlags) |
                             (value.valid_flags & kLauncherFlags));
    PublishGimbalFeedback();
  }
  void OnChassisYaw(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    chassis_imu_yaw_ = value;
    PublishOffline();
  }
  void OnChassisYawValid(const bool& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    chassis_imu_yaw_valid_ = value;
    PublishOffline();
  }
  void OnCapacitorCapacity(const uint8_t& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (value <= 100U) {
      capacitor_capacity_ = value;
    } else {
      capacitor_capacity_valid_ = false;
    }
    PublishOffline();
  }
  void OnCapacitorValid(const bool& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    capacitor_capacity_valid_ = value;
    PublishOffline();
  }
  void OnGimbalYaw(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    yawmotor_angle_ = value;
    gimbal_feedback_.yaw = value;
    gimbal_feedback_.valid_flags |= Pldx::NavHostData::FEEDBACK_YAW_VALID;
    PublishGimbalFeedback();
    PublishOffline();
  }
  void OnGimbalPitch(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    gimbal_feedback_.pitch = value;
    gimbal_feedback_.valid_flags |= Pldx::NavHostData::FEEDBACK_PITCH_VALID;
    PublishGimbalFeedback();
  }
  void OnGimbalYawOmega(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    gimbal_feedback_.yaw_velocity = value;
    yaw_omega_time_ = Now();
    yaw_omega_seen_ = true;
    PublishGimbalFeedback();
  }
  void OnGimbalPitchOmega(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    gimbal_feedback_.pitch_velocity = value;
    pitch_omega_time_ = Now();
    pitch_omega_seen_ = true;
    PublishGimbalFeedback();
  }
  void OnGimbalMode(const uint8_t& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    gimbal_feedback_.gimbal_mode = value;
    gimbal_feedback_.valid_flags |=
        Pldx::NavHostData::FEEDBACK_GIMBAL_MODE_VALID;
    PublishGimbalFeedback();
  }
  void OnVisionTask(const uint8_t& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    gimbal_feedback_.vision_task = value;
    gimbal_feedback_.valid_flags |=
        Pldx::NavHostData::FEEDBACK_VISION_TASK_VALID;
    PublishGimbalFeedback();
  }
  void PublishGimbalFeedback() {
    const auto NOW = Now();
    if (Pldx::NavHostDataDetail::AngularVelocityFresh(
            NOW, yaw_omega_time_, pitch_omega_time_, yaw_omega_seen_,
            pitch_omega_seen_) &&
        std::isfinite(gimbal_feedback_.yaw_velocity) &&
        std::isfinite(gimbal_feedback_.pitch_velocity)) {
      gimbal_feedback_.valid_flags |=
          Pldx::NavHostData::FEEDBACK_ANGULAR_VELOCITY_VALID;
    } else {
      gimbal_feedback_.valid_flags = static_cast<uint8_t>(
          gimbal_feedback_.valid_flags &
          static_cast<uint8_t>(
              ~Pldx::NavHostData::FEEDBACK_ANGULAR_VELOCITY_VALID));
    }
    gimbal_feedback_topic_.Publish(gimbal_feedback_);
  }
  void PublishOffline() {
    Pldx::NavHostData::SentryInfoOffline info{};
    info.yaw_camerainit_to_gimbal =
        Pldx::NavHostDataDetail::YawRadToCamerainitDeg(yawmotor_angle_);
    info.lifter_current_pos = 0U;
    info.is_transformable = false;
    info.transform_state = 0.0F;
    info.chassis_imu_yaw = chassis_imu_yaw_valid_
                               ? chassis_imu_yaw_
                               : std::numeric_limits<float>::quiet_NaN();
    // 255 is the ABI-preserving unknown sentinel; 0..100 are percent.
    info.capacitor_capacity =
        capacitor_capacity_valid_ ? capacitor_capacity_ : 255U;
    info.tunnel_yaw_aligned = Pldx::NavHostDataDetail::IsTunnelYawAligned(
        chassis_imu_yaw_, tunnel_align_active_, tunnel_align_angle_rad_,
        chassis_imu_yaw_valid_);
    offline_info_topic_.Publish(info);
  }
  CMD::Data BuildAI(LibXR::MillisecondTimestamp now) {
    CMD::Data value{};
    value.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    if (tracker_.CommandFresh(now)) {
      value.chassis.source = CMD::ChassisCommandSource::NAVIGATION;
      value.chassis.navigation_velocity.vx_mps = latest_chassis_.vx_mps;
      value.chassis.navigation_velocity.vy_mps = latest_chassis_.vy_mps;
      value.chassis.navigation_velocity.wz_rad_s = latest_chassis_.vw_rad_s;
      value.chassis_online = true;
    }
    if (visual_gimbal_seen_ &&
        (now - visual_gimbal_time_).ToMillisecond() <=
            Pldx::NavHostDataDetail::VISUAL_SOURCE_TIMEOUT_MS) {
      value.gimbal.yaw = visual_gimbal_.yaw;
      value.gimbal.pit = visual_gimbal_.pit;
      value.gimbal.yaw_dot = visual_gimbal_.yaw_dot;
      value.gimbal.pit_dot = visual_gimbal_.pit_dot;
      value.gimbal.yaw_ddot = visual_gimbal_.yaw_ddot;
      value.gimbal.pit_ddot = visual_gimbal_.pit_ddot;
      value.gimbal_online = true;
    }
    if (visual_fire_seen_ &&
        (now - visual_fire_time_).ToMillisecond() <=
            Pldx::NavHostDataDetail::VISUAL_SOURCE_TIMEOUT_MS)
      value.launcher.isfire = visual_fire_.isfire;
    return value;
  }
  void OnReferee(const Referee::RobotGameRefereePack& source) {
    // Referee publishes raw uint32_t receive times; 0 means never received.
    const auto NOW = Now();
    const bool hp_fresh =
        source.robot_hp_received_time_ms != 0U &&
        (NOW - LibXR::MillisecondTimestamp(source.robot_hp_received_time_ms))
                .ToMillisecond() <= 250U;
    const bool pos_fresh =
        source.sentry_pos_received_time_ms != 0U &&
        (NOW - LibXR::MillisecondTimestamp(source.sentry_pos_received_time_ms))
                .ToMillisecond() <= 250U;
    Pldx::NavHostData::GameInfo game{};
    game.game_time_remaining = source.game_status.stage_remain_time;
    game.coin_remaining = source.bullet_remain.coin_remain;
    game.game_status = static_cast<uint8_t>(source.game_status.game_progress);
    LibXR::Memory::FastCopy(&game.event_code, &source.field_event,
                            sizeof(game.event_code));
    game.enemy_outpost_hp = source.robot_hp.enemy_outpost_hp;
    game.enemy_base_hp = source.robot_hp.enemy_base_hp;
    game_info_topic_.Publish(game);

    Pldx::NavHostData::SentryInfoOnline online{};
    online.self_health = source.robot_status.remain_hp;
    online.bullets_remaining = source.bullet_remain.bullet_17_remain;
    online.cooling_value = source.robot_status.shooter_cooling_value;
    online.heat_limit = source.robot_status.shooter_heat_limit;
    online.current_heat = source.power_heat.launcher_id1_17_heat;
    online.sentry_pos_x = source.robot_pos.x;
    online.sentry_pos_y = source.robot_pos.y;
    online.speed_monitor_angle = source.robot_pos.angle;
    const auto& sentry = source.sentry_info;
    online.sentry_info_1 = (sentry.exchanged_bullet_num & 0x7FFU) |
                           ((sentry.exchanged_bullet_times & 0x0FU) << 11U) |
                           ((sentry.exchanged_blood_times & 0x0FU) << 15U) |
                           ((sentry.could_risen_free & 0x01U) << 19U) |
                           ((sentry.could_risen_exchanged & 0x01U) << 20U) |
                           ((sentry.risen_cost & 0x03FFU) << 21U);
    online.sentry_info_2 =
        static_cast<uint16_t>((sentry.current_state & 0x03U) |
                              ((sentry.own_mech_state & 0x01U) << 2U));
    // info_3 posture-buff timers are not present in the referee source.
    online.energy_ratio = source.robot_buff.percent_100_remain_energy  ? 100U
                          : source.robot_buff.percent_50_remain_energy ? 50U
                                                                       : 0U;
    online_info_topic_.Publish(online);

    const uint8_t id = source.robot_status.robot_id;
    const bool blue = id >= 101U && id <= 199U;
    const bool ids_valid = blue || (id >= 1U && id <= 99U);
    if ((source.source_valid_mask & Referee::SOURCE_ROBOT_HP) != 0U &&
        (source.source_valid_mask & Referee::SOURCE_SENTRY_POS) != 0U &&
        hp_fresh && pos_fresh && ids_valid) {
      Pldx::NavHostData::TeamInfo info{};
      const uint8_t ids[4] = {static_cast<uint8_t>(blue ? 101U : 1U),
                              static_cast<uint8_t>(blue ? 102U : 2U),
                              static_cast<uint8_t>(blue ? 103U : 3U),
                              static_cast<uint8_t>(blue ? 104U : 4U)};
      const uint16_t hp[4] = {
          source.robot_hp.ally_1_robot_hp, source.robot_hp.ally_2_robot_hp,
          source.robot_hp.ally_3_robot_hp, source.robot_hp.ally_4_robot_hp};
      const float xy[8] = {
          source.sentry_pos.hero_x,       source.sentry_pos.hero_y,
          source.sentry_pos.engineer_x,   source.sentry_pos.engineer_y,
          source.sentry_pos.standard_3_x, source.sentry_pos.standard_3_y,
          source.sentry_pos.standard_4_x, source.sentry_pos.standard_4_y};
      for (size_t i = 0; i < 4; ++i)
        info.ally_status[i] = {ids[i], hp[i], xy[2 * i], xy[2 * i + 1]};
      info.outpost_hp = source.robot_hp.ally_outpost_hp;
      info.base_hp = source.robot_hp.ally_base_hp;
      team_info_topic_.Publish(info);
    }

    Pldx::NavHostData::RadarInfo radar{};
    radar.enemy_coin_left = source.bullet_remain.coin_remain;
    radar_info_topic_.Publish(radar);
  }
  class Sink {
   public:
    explicit Sink(NavHostData& owner) : owner_(owner) {}
    void PublishBuyBulletNum(uint16_t v) {
      owner_.buy_bullet_topic_.Publish(v);
    }
    void PublishRemoteBuyBulletTimes(uint8_t v) {
      owner_.remote_bullet_topic_.Publish(v);
    }
    void PublishRemoteBuyHpTimes(uint8_t v) {
      owner_.remote_hp_topic_.Publish(v);
    }
    void PublishBuyResurrection(bool v) {
      owner_.resurrection_topic_.Publish(v);
    }

   private:
    NavHostData& owner_;
  };
  CMD* cmd_;
  LibXR::UART* uart_;
  LibXR::Mutex mutex_;
  LibXR::Topic chassis_topic_, behavior_topic_, team_info_topic_,
      game_info_topic_, online_info_topic_, offline_info_topic_,
      radar_info_topic_, gimbal_feedback_topic_, nav_chassis_mode_topic_,
      use_capacitor_topic_;
  LibXR::Topic buy_bullet_topic_{
      LibXR::Topic::CreateTopic<uint16_t>("sentry_buy_bullet_num")};
  LibXR::Topic remote_bullet_topic_{
      LibXR::Topic::CreateTopic<uint8_t>("sentry_remote_buy_bullet_times")};
  LibXR::Topic remote_hp_topic_{
      LibXR::Topic::CreateTopic<uint8_t>("sentry_remote_buy_hp_times")};
  LibXR::Topic resurrection_topic_{
      LibXR::Topic::CreateTopic<bool>("sentry_buy_resurrection")};
  Pldx::NavHostData::ChassisTarget latest_chassis_{};
  Pldx::NavHostDataDetail::HostGimbalTarget visual_gimbal_{};
  Pldx::NavHostDataDetail::HostFireNotify visual_fire_{};
  LibXR::MillisecondTimestamp visual_gimbal_time_{}, visual_fire_time_{};
  bool visual_gimbal_seen_{}, visual_fire_seen_{};
  float yawmotor_angle_{};
  LibXR::MillisecondTimestamp yaw_omega_time_{};
  LibXR::MillisecondTimestamp pitch_omega_time_{};
  bool yaw_omega_seen_{};
  bool pitch_omega_seen_{};
  float chassis_imu_yaw_{};
  bool chassis_imu_yaw_valid_{};
  uint8_t capacitor_capacity_{};
  bool capacitor_capacity_valid_{};
  bool tunnel_align_active_{};
  float tunnel_align_angle_rad_{};
  Pldx::NavHostData::GimbalFeedback gimbal_feedback_{};
  Pldx::NavHostDataDetail::ChassisCommandTracker tracker_;
  Pldx::NavHostDataDetail::DecisionSequenceTracker decision_;
};
#endif
