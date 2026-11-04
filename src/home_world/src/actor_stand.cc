// Switches an actor between a walking and a standing animation.
// FollowActor moves the actor (TrajectoryPose) but freezes it mid-stride when
// it stops; this system shows the standing animation while the pose stays put.
// While <keep_walking_topic> says true (evacuation follow), it keeps walking in
// place instead: the walking animation time keeps advancing at the walking rate.
#include <atomic>
#include <chrono>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/System.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/components/Actor.hh>
#include <gz/transport/Node.hh>

namespace home_world
{
class ActorStand : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    actor_ = gz::sim::Actor(_entity);
    walking_ = _sdf->Get<std::string>("walking", "walking").first;
    standing_ = _sdf->Get<std::string>("standing", "standing").first;
    still_time_ = _sdf->Get<double>("still_time", 0.3).first;
    const auto topic =
        _sdf->Get<std::string>("keep_walking_topic", "/resident_keep_walking").first;
    node_.Subscribe(topic, &ActorStand::OnKeepWalking, this);
  }

  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    auto pose = actor_.TrajectoryPose(_ecm);
    if (!pose)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    const double dt = std::chrono::duration<double>(_info.dt).count();
    auto *anim_time = _ecm.Component<gz::sim::components::AnimationTime>(actor_.Entity());
    const double anim_now =
        anim_time ? std::chrono::duration<double>(anim_time->Data()).count() : 0.0;
    bool moved = false;
    if (!last_pos_)
      last_pos_ = pose->Pos();  // starts standing
    else if (*last_pos_ != pose->Pos()) {
      last_pos_ = pose->Pos();
      moved_at_ = now;
      moved = true;
    }
    // Walking rate (animation s per sim s) measured while FollowActor moves the actor
    if (moved && dt > 0.0 && anim_now > last_anim_)
      walk_rate_ = (anim_now - last_anim_) / dt;
    const bool still = now - moved_at_ > still_time_;
    const bool keep = keep_walking_.load();
    if (!moved && keep && anim_time && dt > 0.0) {
      // FollowActor leaves the animation frozen while stopped: walk in place
      *anim_time = gz::sim::components::AnimationTime(
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(anim_now + walk_rate_ * dt)));
      _ecm.SetChanged(actor_.Entity(), gz::sim::components::AnimationTime::typeId,
                      gz::sim::ComponentState::OneTimeChange);
    }
    last_anim_ = anim_time ? std::chrono::duration<double>(anim_time->Data()).count() : 0.0;
    const std::string &want = (still && !keep) ? standing_ : walking_;
    if (actor_.AnimationName(_ecm) != want) {
      actor_.SetAnimationName(_ecm, want);
      // SetAnimationName does not mark the change, so the GUI would keep the old animation
      _ecm.SetChanged(actor_.Entity(), gz::sim::components::AnimationName::typeId,
                      gz::sim::ComponentState::OneTimeChange);
    }
  }

private:
  void OnKeepWalking(const gz::msgs::Boolean &_msg) { keep_walking_ = _msg.data(); }

  gz::sim::Actor actor_;
  gz::transport::Node node_;
  std::atomic<bool> keep_walking_{false};
  double walk_rate_{1.0};
  double last_anim_{0.0};
  std::string walking_, standing_;
  double still_time_{0.3};
  std::optional<gz::math::Vector3d> last_pos_;
  double moved_at_{-1e9};
};
}  // namespace home_world

GZ_ADD_PLUGIN(home_world::ActorStand, gz::sim::System,
              home_world::ActorStand::ISystemConfigure,
              home_world::ActorStand::ISystemUpdate)
