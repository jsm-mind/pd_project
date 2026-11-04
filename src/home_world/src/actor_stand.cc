// Switches an actor between a walking and a standing animation.
// FollowActor moves the actor (TrajectoryPose) but freezes it mid-stride when
// it stops; this system shows the standing animation while the pose stays put.
#include <chrono>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/System.hh>

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
    if (!last_pos_)
      last_pos_ = pose->Pos();  // starts standing
    else if (*last_pos_ != pose->Pos()) {
      last_pos_ = pose->Pos();
      moved_at_ = now;
    }
    const std::string &want = (now - moved_at_ > still_time_) ? standing_ : walking_;
    if (actor_.AnimationName(_ecm) != want)
      actor_.SetAnimationName(_ecm, want);
  }

private:
  gz::sim::Actor actor_;
  std::string walking_, standing_;
  double still_time_{0.3};
  std::optional<gz::math::Vector3d> last_pos_;
  double moved_at_{-1e9};
};
}  // namespace home_world

GZ_ADD_PLUGIN(home_world::ActorStand, gz::sim::System,
              home_world::ActorStand::ISystemConfigure,
              home_world::ActorStand::ISystemUpdate)
