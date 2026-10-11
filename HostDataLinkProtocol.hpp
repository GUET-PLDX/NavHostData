#pragma once
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"

namespace Pldx::HostDataLink {

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

// Compact launcher sample consumed by HostDataLink before publishing the
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

}  // namespace Pldx::HostDataLink

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

namespace Pldx::HostDataLinkDetail {

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

inline bool ChassisCommandValid(const HostDataLink::ChassisTarget& command) {
  return std::isfinite(command.vx_mps) && std::isfinite(command.vy_mps) &&
         std::isfinite(command.vw_rad_s) &&
         std::isfinite(command.current_yaw) &&
         std::isfinite(command.current_vx) &&
         std::isfinite(command.current_vy) &&
         std::isfinite(command.current_vw) && std::isfinite(command.delta_yaw);
}

class ChassisCommandTracker {
 public:
  bool Observe(const HostDataLink::ChassisTarget& command,
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
  DecisionPublication Decode(const HostDataLink::BehaviorData& value,
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

}  // namespace Pldx::HostDataLinkDetail
