#include <cassert>
#include <cmath>
#include <limits>

#include "../NavHostData.hpp"

using Pldx::NavHostData::BehaviorData;
using Pldx::NavHostData::ChassisTarget;
using Pldx::NavHostDataDetail::AngularVelocityFresh;
using Pldx::NavHostDataDetail::AUTO_CHASSIS_MODE_NAVIGATION;
using Pldx::NavHostDataDetail::AUTO_CHASSIS_MODE_ROTOR;
using Pldx::NavHostDataDetail::ChassisCommandTracker;
using Pldx::NavHostDataDetail::ChassisCommandValid;
using Pldx::NavHostDataDetail::DecisionSequenceTracker;
using Pldx::NavHostDataDetail::FEEDBACK_OMEGA_TIMEOUT_MS;
using Pldx::NavHostDataDetail::IsTunnelYawAligned;
using Pldx::NavHostDataDetail::YawRadToCamerainitDeg;

void ChassisWatchdogAndImmediateRecovery() {
  ChassisCommandTracker tracker;
  ChassisTarget moving{};
  moving.vx_mps = 1.0F;
  assert(tracker.Observe(moving, 0));
  assert(tracker.LastSequence() == 1U);
  assert(tracker.CommandFresh(149));

  ChassisTarget invalid = moving;
  invalid.vw_rad_s = std::numeric_limits<float>::infinity();
  assert(!tracker.Observe(invalid, 100));
  assert(tracker.RejectedCount() == 1U);
  assert(tracker.CommandFresh(100));

  assert(tracker.Observe(moving, 101));
  assert(tracker.LastSequence() == 2U);
  assert(tracker.CommandFresh(101));

  assert(!tracker.CommandFresh(251));
  assert(tracker.Observe(moving, 252));
  assert(tracker.LastSequence() == 3U);
  assert(tracker.CommandFresh(252));
}

void ChassisNavigationSpeedValidation() {
  ChassisTarget finite{};
  finite.vx_mps = 30.0F;
  finite.vy_mps = -30.0F;
  finite.vw_rad_s = 30.0F;
  assert(ChassisCommandValid(finite));

  finite.vw_rad_s = std::numeric_limits<float>::infinity();
  assert(!ChassisCommandValid(finite));
}

void BehaviorCumulativeDeltas() {
  DecisionSequenceTracker tracker;
  BehaviorData value{};
  value.scan_yaw_min_rad = 1.0F;
  value.scan_yaw_max_rad = 2.0F;
  assert(tracker.Decode(value).accepted);
  value.ammo_purchase_request = 10;
  value.remote_ammo_request = 2;
  value.remote_health_request = 1;
  value.revive_request = 1;
  auto publication = tracker.Decode(value);
  assert(publication.publish_buy_bullet_num &&
         publication.buy_bullet_num == 10);
  assert(publication.publish_remote_buy_bullet_times &&
         publication.remote_buy_bullet_times == 2);
  assert(publication.publish_remote_buy_hp_times &&
         publication.remote_buy_hp_times == 1);
  assert(publication.publish_buy_resurrection);
  assert(!tracker.Decode(value).publish_buy_bullet_num);
  value.ammo_purchase_request = 1;
  assert(!tracker.Decode(value).publish_buy_bullet_num);
}

void TunnelYawAlignment() {
  assert(IsTunnelYawAligned(0.0F, true, 0.0F, true));
  assert(IsTunnelYawAligned(3.13F, true, 3.12413936F, true));
  assert(!IsTunnelYawAligned(0.2F, true, 0.0F, true));
  assert(!IsTunnelYawAligned(0.0F, false, 0.0F, true));
  assert(!IsTunnelYawAligned(0.0F, true, 0.0F, false));
}

void AngularVelocityRequiresBothAxesFresh() {
  assert(!AngularVelocityFresh(50, 50, 50, false, true));
  assert(!AngularVelocityFresh(50, 50, 50, true, false));
  assert(AngularVelocityFresh(50, 50, 50, true, true));
  assert(AngularVelocityFresh(50, 0, 0, true, true));
  assert(!AngularVelocityFresh(50 + FEEDBACK_OMEGA_TIMEOUT_MS + 1U, 0, 50, true,
                               true));
  assert(!AngularVelocityFresh(50 + FEEDBACK_OMEGA_TIMEOUT_MS + 1U, 50, 0, true,
                               true));
}

void EncoderYawWrapsToOpenClosedDegrees() {
  assert(YawRadToCamerainitDeg(0.0F) == 0.0F);
  assert(std::fabs(YawRadToCamerainitDeg(static_cast<float>(LibXR::PI) / 2.0F) -
                   90.0F) < 1.0e-4F);
  assert(std::fabs(YawRadToCamerainitDeg(static_cast<float>(LibXR::PI)) -
                   180.0F) < 1.0e-4F);
  assert(std::fabs(YawRadToCamerainitDeg(-static_cast<float>(LibXR::PI)) -
                   180.0F) < 1.0e-4F);
  assert(std::fabs(YawRadToCamerainitDeg(static_cast<float>(LibXR::TWO_PI))) <
         1.0e-4F);
  assert(YawRadToCamerainitDeg(std::numeric_limits<float>::quiet_NaN()) ==
         0.0F);
}

int main() {
  static_assert(sizeof(ChassisTarget) == 45U);
  static_assert(sizeof(BehaviorData) == 26U);
  static_assert(AUTO_CHASSIS_MODE_ROTOR == 2U);
  static_assert(AUTO_CHASSIS_MODE_NAVIGATION == 4U);
  ChassisWatchdogAndImmediateRecovery();
  ChassisNavigationSpeedValidation();
  BehaviorCumulativeDeltas();
  TunnelYawAlignment();
  EncoderYawWrapsToOpenClosedDegrees();
  AngularVelocityRequiresBothAxesFresh();
}
