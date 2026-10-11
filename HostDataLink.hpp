#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ROS2_LIBXR navigation link protocol and host endpoint
depends:
- pldx/CMD@master
- pldx/Referee@master
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
#include "HostDataLinkProtocol.hpp"
#include "Referee.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "timebase.hpp"
#include "uart.hpp"

class HostDataLink {
 public:
  HostDataLink(LibXR::UART& uart, CMD& cmd,
               const char* referee_topic_name = "sentry_ref",
               const char* pitch_topic_name = "pitchmotor_angle",
               const char* yaw_topic_name = "yawmotor_angle",
               const char* quaternion_topic_name = "ahrs_quaternion")
      : cmd_(&cmd),
        uart_(&uart),
        chassis_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::ChassisTarget>(
                "nav_data")),
        behavior_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::BehaviorData>(
                "behavior_data")),
        team_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::TeamInfo>(
                "team_info")),
        game_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::GameInfo>(
                "game_info")),
        online_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::SentryInfoOnline>(
                "online_info")),
        offline_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::SentryInfoOffline>(
                "offline_info")),
        radar_info_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::RadarInfo>(
                "radar_info")),
        gimbal_feedback_topic_(
            LibXR::Topic::CreateTopic<Pldx::HostDataLink::GimbalFeedback>(
                Pldx::HostDataLink::GIMBAL_FEEDBACK_TOPIC)),
        nav_chassis_mode_topic_(LibXR::Topic::CreateTopic<uint32_t>(
            "nav_chassis_mode", nullptr, false)),
        use_capacitor_topic_(
            LibXR::Topic::CreateTopic<bool>("use_capacitor", nullptr, false)) {
    ASSERT(uart_->SetConfig({921600U, LibXR::UART::Parity::NO_PARITY, 8U,
                             1U}) == LibXR::ErrorCode::OK);
    RegisterRawCallback<&HostDataLink::OnChassis>(chassis_topic_);
    RegisterRawCallback<&HostDataLink::OnBehavior>(behavior_topic_);
    RegisterNamedCallback<Referee::RobotGameRefereePack,
                          &HostDataLink::OnReferee>(referee_topic_name);
    RegisterNamedCallback<Pldx::HostDataLinkDetail::HostGimbalTarget,
                          &HostDataLink::OnVisualGimbal>(
        Pldx::HostDataLink::TARGET_EULER_TOPIC);
    RegisterNamedCallback<Pldx::HostDataLinkDetail::HostFireNotify,
                          &HostDataLink::OnVisualFire>(
        Pldx::HostDataLink::FIRE_NOTIFY_TOPIC);
    RegisterNamedCallback<Pldx::HostDataLink::GimbalFeedbackV1,
                          &HostDataLink::OnLauncherFeedback>(
        Pldx::HostDataLink::LAUNCHER_FEEDBACK_TOPIC);
    RegisterNamedCallback<float, &HostDataLink::OnGimbalYaw>(yaw_topic_name);
    RegisterNamedCallback<float, &HostDataLink::OnGimbalPitch>(
        pitch_topic_name);
    RegisterNamedCallback<float, &HostDataLink::OnGimbalYawOmega>(
        "yawmotor_omega");
    RegisterNamedCallback<float, &HostDataLink::OnGimbalPitchOmega>(
        "pitchmotor_omega");
    RegisterNamedCallback<uint8_t, &HostDataLink::OnGimbalMode>("gimbal_mode");
    RegisterNamedCallback<uint8_t, &HostDataLink::OnVisionTask>("vision_task");
    RegisterNamedCallback<float, &HostDataLink::OnChassisYaw>(
        "chassis_imu_yaw");
    RegisterNamedCallback<bool, &HostDataLink::OnChassisYawValid>(
        "chassis_imu_yaw_valid");
    RegisterNamedCallback<uint8_t, &HostDataLink::OnCapacitorCapacity>(
        "chassis_capacitor_capacity");
    RegisterNamedCallback<bool, &HostDataLink::OnCapacitorValid>(
        "chassis_capacitor_valid");
    UNUSED(pitch_topic_name);
    UNUSED(yaw_topic_name);
    UNUSED(quaternion_topic_name);
  }
  void OnMonitor() {}

 private:
  static LibXR::MillisecondTimestamp Now() {
    return LibXR::Timebase::GetMilliseconds();
  }
  template <typename Data, void (HostDataLink::*Method)(const Data&)>
  void RegisterCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool, HostDataLink* self, const Data& data) {
          (self->*Method)(data);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <
      void (HostDataLink::*Method)(const Pldx::HostDataLink::ChassisTarget&)>
  void RegisterRawCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool, HostDataLink* self,
           const LibXR::Topic::RawMessageView& message) {
          if (message.timestamp == 0U || message.payload.addr_ == nullptr ||
              message.payload.size_ !=
                  sizeof(Pldx::HostDataLink::ChassisTarget)) {
            return;
          }
          Pldx::HostDataLink::ChassisTarget value{};
          LibXR::Memory::FastCopy(&value, message.payload.addr_, sizeof(value));
          (self->*Method)(value);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <
      void (HostDataLink::*Method)(const Pldx::HostDataLink::BehaviorData&)>
  void RegisterRawCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool, HostDataLink* self,
           const LibXR::Topic::RawMessageView& message) {
          if (message.timestamp == 0U || message.payload.addr_ == nullptr ||
              message.payload.size_ !=
                  sizeof(Pldx::HostDataLink::BehaviorData)) {
            return;
          }
          Pldx::HostDataLink::BehaviorData value{};
          LibXR::Memory::FastCopy(&value, message.payload.addr_, sizeof(value));
          (self->*Method)(value);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <typename Data, void (HostDataLink::*Method)(const Data&)>
  void RegisterNamedCallback(const char* name) {
    LibXR::Topic topic(LibXR::Topic::FindOrCreate<Data>(name));
    RegisterCallback<Data, Method>(topic);
  }
  void OnChassis(const Pldx::HostDataLink::ChassisTarget& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    const auto NOW = Now();
    if (tracker_.Observe(value, NOW)) {
      latest_chassis_ = value;
    }
    cmd_->FeedAI(BuildAI(NOW));
  }
  void OnBehavior(const Pldx::HostDataLink::BehaviorData& value) {
    Pldx::HostDataLinkDetail::DecisionPublication publication;
    bool publish_chassis_mode = false;
    uint32_t chassis_mode =
        Pldx::HostDataLinkDetail::AUTO_CHASSIS_MODE_NAVIGATION;
    {
      LibXR::Mutex::LockGuard lock(mutex_);
      tunnel_align_active_ = value.tunnel_align_active;
      tunnel_align_angle_rad_ = value.tunnel_align_angle_rad;
      PublishOffline();
      publication = decision_.Decode(value);
      publish_chassis_mode = cmd_->GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL;
      chassis_mode =
          value.use_gyro_mode
              ? Pldx::HostDataLinkDetail::AUTO_CHASSIS_MODE_ROTOR
              : Pldx::HostDataLinkDetail::AUTO_CHASSIS_MODE_NAVIGATION;
    }
    bool use_capacitor = value.use_capacitor;
    use_capacitor_topic_.Publish(use_capacitor);
    if (publish_chassis_mode) {
      nav_chassis_mode_topic_.Publish(chassis_mode);
    }
    Sink sink(*this);
    Pldx::HostDataLinkDetail::DispatchDecision(publication, sink);
  }
  void OnVisualGimbal(const Pldx::HostDataLinkDetail::HostGimbalTarget& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    visual_gimbal_ = value;
    visual_gimbal_time_ = Now();
    visual_gimbal_seen_ = true;
    cmd_->FeedAI(BuildAI(visual_gimbal_time_));
  }
  void OnVisualFire(const Pldx::HostDataLinkDetail::HostFireNotify& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    visual_fire_ = value;
    visual_fire_time_ = Now();
    visual_fire_seen_ = true;
    cmd_->FeedAI(BuildAI(visual_fire_time_));
  }
  void OnLauncherFeedback(const Pldx::HostDataLink::GimbalFeedbackV1& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if ((value.valid_flags &
         static_cast<uint8_t>(~Pldx::HostDataLink::FEEDBACK_VALID_MASK)) != 0U)
      return;
    gimbal_feedback_.bullet_speed = value.bullet_speed_mps;
    gimbal_feedback_.bullet_count = value.bullet_count;
    constexpr uint8_t kLauncherFlags =
        Pldx::HostDataLink::FEEDBACK_BULLET_SPEED_VALID |
        Pldx::HostDataLink::FEEDBACK_BULLET_COUNT_VALID;
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
    gimbal_feedback_.valid_flags |= Pldx::HostDataLink::FEEDBACK_YAW_VALID;
    PublishGimbalFeedback();
    PublishOffline();
  }
  void OnGimbalPitch(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    gimbal_feedback_.pitch = value;
    gimbal_feedback_.valid_flags |= Pldx::HostDataLink::FEEDBACK_PITCH_VALID;
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
        Pldx::HostDataLink::FEEDBACK_GIMBAL_MODE_VALID;
    PublishGimbalFeedback();
  }
  void OnVisionTask(const uint8_t& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    gimbal_feedback_.vision_task = value;
    gimbal_feedback_.valid_flags |=
        Pldx::HostDataLink::FEEDBACK_VISION_TASK_VALID;
    PublishGimbalFeedback();
  }
  void PublishGimbalFeedback() {
    const auto NOW = Now();
    if (Pldx::HostDataLinkDetail::AngularVelocityFresh(
            NOW, yaw_omega_time_, pitch_omega_time_, yaw_omega_seen_,
            pitch_omega_seen_) &&
        std::isfinite(gimbal_feedback_.yaw_velocity) &&
        std::isfinite(gimbal_feedback_.pitch_velocity)) {
      gimbal_feedback_.valid_flags |=
          Pldx::HostDataLink::FEEDBACK_ANGULAR_VELOCITY_VALID;
    } else {
      gimbal_feedback_.valid_flags = static_cast<uint8_t>(
          gimbal_feedback_.valid_flags &
          static_cast<uint8_t>(
              ~Pldx::HostDataLink::FEEDBACK_ANGULAR_VELOCITY_VALID));
    }
    gimbal_feedback_topic_.Publish(gimbal_feedback_);
  }
  void PublishOffline() {
    Pldx::HostDataLink::SentryInfoOffline info{};
    info.yaw_camerainit_to_gimbal =
        Pldx::HostDataLinkDetail::YawRadToCamerainitDeg(yawmotor_angle_);
    info.lifter_current_pos = 0U;
    info.is_transformable = false;
    info.transform_state = 0.0F;
    info.chassis_imu_yaw = chassis_imu_yaw_valid_
                               ? chassis_imu_yaw_
                               : std::numeric_limits<float>::quiet_NaN();
    // 255 is the ABI-preserving unknown sentinel; 0..100 are percent.
    info.capacitor_capacity =
        capacitor_capacity_valid_ ? capacitor_capacity_ : 255U;
    info.tunnel_yaw_aligned = Pldx::HostDataLinkDetail::IsTunnelYawAligned(
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
            Pldx::HostDataLinkDetail::VISUAL_SOURCE_TIMEOUT_MS) {
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
            Pldx::HostDataLinkDetail::VISUAL_SOURCE_TIMEOUT_MS)
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
    Pldx::HostDataLink::GameInfo game{};
    game.game_time_remaining = source.game_status.stage_remain_time;
    game.coin_remaining = source.bullet_remain.coin_remain;
    game.game_status = static_cast<uint8_t>(source.game_status.game_progress);
    LibXR::Memory::FastCopy(&game.event_code, &source.field_event,
                            sizeof(game.event_code));
    game.enemy_outpost_hp = source.robot_hp.enemy_outpost_hp;
    game.enemy_base_hp = source.robot_hp.enemy_base_hp;
    game_info_topic_.Publish(game);

    Pldx::HostDataLink::SentryInfoOnline online{};
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
      Pldx::HostDataLink::TeamInfo info{};
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

    Pldx::HostDataLink::RadarInfo radar{};
    radar.enemy_coin_left = source.bullet_remain.coin_remain;
    radar_info_topic_.Publish(radar);
  }
  class Sink {
   public:
    explicit Sink(HostDataLink& owner) : owner_(owner) {}
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
    HostDataLink& owner_;
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
  Pldx::HostDataLink::ChassisTarget latest_chassis_{};
  Pldx::HostDataLinkDetail::HostGimbalTarget visual_gimbal_{};
  Pldx::HostDataLinkDetail::HostFireNotify visual_fire_{};
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
  Pldx::HostDataLink::GimbalFeedback gimbal_feedback_{};
  Pldx::HostDataLinkDetail::ChassisCommandTracker tracker_;
  Pldx::HostDataLinkDetail::DecisionSequenceTracker decision_;
};
