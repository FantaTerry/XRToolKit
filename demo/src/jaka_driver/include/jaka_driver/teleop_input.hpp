#ifndef JAKA_DRIVER_TELEOP_INPUT_HPP_
#define JAKA_DRIVER_TELEOP_INPUT_HPP_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <vector>
#include "jaka_driver/teleop_control.hpp"

namespace jaka_teleop {
struct InputFrame {
  ControllerPose pose;
  int64_t stamp_ns = 0;
  double pose_receive = -1, joy_receive = -1;
  double deadline = -1, grip_value = 0, trigger = 0;
  bool pressed = false;
};

// Bounded same-source pairing. All times are supplied by the caller so expiry,
// interleaving and source epoch tests do not depend on wall-clock sleeps.
class InputAssembler {
 public:
  InputAssembler(double source_age, double receive_age, bool require_tracking=true,
                 size_t capacity=32,double release_threshold=.4,double trigger_deadband=.02)
  : source_age_(source_age), receive_age_(receive_age), release_threshold_(release_threshold),
    trigger_deadband_(trigger_deadband),
    require_tracking_(require_tracking), capacity_(capacity) {}

  // Translate the source's REMAINING lifetime once into monotonic time. Later
  // checks cannot accidentally grant a second full receive-age allowance.
  double deadline(double stamp, double ros_now, double mono_now) const {
    const double age=ros_now-stamp;
    if (!std::isfinite(stamp) || stamp<=0 || !std::isfinite(age) || age<-.05 || age>source_age_)
      return -1;
    return mono_now+std::min(receive_age_,source_age_-std::max(0.,age));
  }
  // Source-generated epoch marker, NOT the receiver's clock or largest seen
  // frame. Positive = fixed first recovered source-frame stamp; negative =
  // fixed first loss stamp. Every healthy heartbeat repeats its recovery floor.
  bool tracking_state(int64_t signed_stamp,double now) {
    if (!signed_stamp || signed_stamp==std::numeric_limits<int64_t>::min() ||
        !std::isfinite(now) || now<0) { invalidate();tracked_=false;return false; }
    const bool valid=signed_stamp>0;
    const int64_t stamp=valid ? signed_stamp : -signed_stamp;
    // With one ordered source-state publisher, a backwards event clock is not
    // a new epoch. Stop conservatively; only a strictly newer recovery barrier
    // can authorize inputs again. Do not synthesize a receiver-side timestamp.
    if(stamp<state_stamp_) { invalidate();tracked_=false;return false; }
    if(stamp==state_stamp_ && valid!=tracked_) {
      if(valid) return false;
      invalidate();tracked_=false;tracking_time_=now;return true;
    }
    if(!valid) {
      if(stamp>state_stamp_ || tracked_) invalidate();
      floor_=std::max(floor_,stamp);
    } else if(stamp>recovery_barrier_) {
      invalidate();
      recovery_barrier_=stamp;
      // The first recovered frame itself is eligible; every pre-epoch frame
      // is rejected, even if both its Pose and Joy were previously unseen.
      floor_=std::max(floor_,stamp-1);
    }
    state_stamp_=stamp; tracked_=valid; tracking_time_=now;
    return true;
  }
  void invalidate() {
    pending_.clear(); ready_.clear(); current_valid_=false;
    floor_=std::max(floor_,seen_); interrupted_=true;
    revoked_through_=std::max(revoked_through_,seen_);
    trigger_requests_.clear();
    trigger_retired_through_=std::max(trigger_retired_through_,seen_);
  }
  bool take_interruption() { bool v=interrupted_; interrupted_=false; return v; }

  void pose(int64_t stamp,const ControllerPose &p,double ros_now,double mono_now) {
    if (superseded(stamp)) return;
    const double limit=deadline(stamp*1e-9,ros_now,mono_now);
    Quaternion normalized;
    if (limit<mono_now || !std::isfinite(p.tran.x) || !std::isfinite(p.tran.y) ||
        !std::isfinite(p.tran.z) || !quat_normalize(p.quat,&normalized)) { invalidate();return; }
    if (!accept(stamp,mono_now)) return;
    auto &part=pending_[stamp];
    if (part.pose) return;
    part.pose=true; part.frame.pose=p; part.frame.pose.quat=normalized;
    part.frame.pose_receive=mono_now; part.pose_deadline=limit;
    complete(stamp);
  }
  void joy(int64_t stamp,double grip,double trigger,bool grip_valid,bool trigger_valid,
           double ros_now,double mono_now) {
    // A source stamp identifies one immutable candidate request. Neither a
    // duplicate nor an evicted record may recreate a cleared revocation latch.
    if (superseded(stamp) || stamp<=trigger_retired_through_ || trigger_requests_.count(stamp)) return;
    const double limit=deadline(stamp*1e-9,ros_now,mono_now);
    if (limit<mono_now || !grip_valid || !trigger_valid || !std::isfinite(grip) ||
        grip<0 || grip>1 || !std::isfinite(trigger) || trigger<0 || trigger>1) { invalidate();return; }
    if (!accept(stamp,mono_now)) return;
    record_trigger_request(stamp,trigger);
    if (stamp>latest_joy_stamp_) {
      if(grip<=release_threshold_ && latest_grip_>release_threshold_) interrupted_=true;
      latest_joy_stamp_=stamp; latest_grip_=grip;
      latest_trigger_=trigger; latest_joy_deadline_=limit;
    }
    auto &part=pending_[stamp];
    if (part.joy) return;
    part.joy=true; part.frame.grip_value=grip; part.frame.trigger=trigger;
    part.frame.joy_receive=mono_now; part.joy_deadline=limit;
    complete(stamp);
  }
  std::vector<InputFrame> drain(double now) {
    std::vector<InputFrame> frames;
    for (const auto &entry:ready_) {
      const auto &f=entry.second;
      if (f.stamp_ns<=floor_) continue;
      floor_=f.stamp_ns;
      if (fresh(f,now)) { frames.push_back(f); current_=f; current_valid_=true; }
    }
    ready_.clear();
    // Old partials cannot pair after a newer complete frame has been consumed.
    for(auto it=pending_.begin();it!=pending_.end();) {
      if(it->first<=floor_ || std::min(it->second.pose_deadline,it->second.joy_deadline)<now)
        it=pending_.erase(it);
      else ++it;
    }
    return frames;
  }
  bool fresh(const InputFrame &f,double now) const {
    return now<=f.deadline && now>=f.pose_receive && now>=f.joy_receive &&
      tracked_ && (!require_tracking_ || (now>=tracking_time_ && now-tracking_time_<=receive_age_));
  }
  bool current_fresh(double now) const { return current_valid_ && fresh(current_,now); }
  bool release_fresh(double now,double release_threshold) const {
    return current_fresh(now) && current_.grip_value<=release_threshold && latest_grip_<=release_threshold;
  }
  // Withdrawal is effective on a validated NEW Joy, before its Pose arrives.
  // A change followed by a change back still revokes the old request: only a
  // later complete frame may authorize the replacement. Replays cannot revoke.
  bool trigger_request_current(int64_t request_stamp,double trigger,double deadband,double now) const {
    const auto request=trigger_requests_.find(request_stamp);
    if(request==trigger_requests_.end() || request->second.trigger!=trigger ||
        deadband!=trigger_deadband_ || request->second.revoked) return false;
    return request_stamp>=recovery_barrier_ && request_stamp>revoked_through_ &&
      latest_joy_stamp_>=request_stamp &&
      now<=latest_joy_deadline_ && std::abs(latest_trigger_-trigger)<deadband &&
      tracked_ && (!require_tracking_ || (now>=tracking_time_ && now-tracking_time_<=receive_age_));
  }
  bool tracking_valid() const { return tracked_; }
  size_t pending_size() const { return pending_.size(); }
  size_t ready_size() const { return ready_.size(); }
  size_t dropped() const { return dropped_; }
 private:
  struct TriggerRequest {
    double trigger;
    bool revoked=false;
  };
  void record_trigger_request(int64_t stamp,double trigger) {
    TriggerRequest candidate{trigger,false};
    for(auto &entry:trigger_requests_) {
      if(std::abs(trigger-entry.second.trigger)<trigger_deadband_) continue;
      if(entry.first<stamp) entry.second.revoked=true;
      else if(entry.first>stamp) candidate.revoked=true;
    }
    trigger_requests_.emplace(stamp,candidate);
    // Bounded request history. Eviction cancels only that old request; it must
    // not interrupt ordinary motion or allow a replay to register it again.
    while(trigger_requests_.size()>capacity_) {
      trigger_retired_through_=std::max(trigger_retired_through_,trigger_requests_.begin()->first);
      trigger_requests_.erase(trigger_requests_.begin());
    }
  }
  struct Partial {
    InputFrame frame;
    bool pose=false,joy=false;
    double pose_deadline=std::numeric_limits<double>::infinity();
    double joy_deadline=std::numeric_limits<double>::infinity();
  };
  bool accept(int64_t stamp,double now) {
    seen_=std::max(seen_,stamp);
    if (stamp<=floor_ || ready_.count(stamp)) return false;
    if (!tracked_ || (require_tracking_ && now-tracking_time_>receive_age_)) return false;
    return true;
  }
  bool superseded(int64_t stamp) const {
    return stamp<=floor_ || ready_.count(stamp) || (!ready_.empty() && stamp<ready_.rbegin()->first);
  }
  void complete(int64_t stamp) {
    auto it=pending_.find(stamp);
    if (it->second.pose && it->second.joy) {
      auto f=it->second.frame; f.stamp_ns=stamp;
      f.deadline=std::min(it->second.pose_deadline,it->second.joy_deadline);
      ready_[stamp]=f; pending_.erase(it);
    }
    while (pending_.size()>capacity_) { pending_.erase(pending_.begin()); ++dropped_; interrupted_=true; }
    while (ready_.size()>capacity_) { ready_.erase(ready_.begin()); ++dropped_; interrupted_=true; }
  }
  double source_age_,receive_age_,release_threshold_,trigger_deadband_,tracking_time_=-1,latest_grip_=1;
  double latest_trigger_=0,latest_joy_deadline_=-1;
  bool require_tracking_,tracked_=false,interrupted_=false,current_valid_=false;
  size_t capacity_,dropped_=0;
  int64_t floor_=0,seen_=0,latest_joy_stamp_=0;
  int64_t state_stamp_=0,recovery_barrier_=0,revoked_through_=0,trigger_retired_through_=0;
  std::map<int64_t,TriggerRequest> trigger_requests_;
  std::map<int64_t,Partial> pending_;
  std::map<int64_t,InputFrame> ready_;
  InputFrame current_;
};
}
#endif
