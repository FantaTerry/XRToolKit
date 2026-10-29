#include <cassert>
#include <iostream>
#include <limits>
#include "jaka_driver/teleop_input.hpp"
using namespace jaka_teleop;
void initial_state(InputAssembler &a) { assert(a.tracking_state(100000000000LL,10)); a.take_interruption(); }
int main() {
  ControllerPose p;
  {
    InputAssembler a(.2,.2); initial_state(a);
    a.pose(100000000000LL,p,100.15,10); a.joy(100000000000LL,0,.8,true,true,100.15,10);
    auto f=a.drain(10); assert(f.size()==1);
    const double expiry=f[0].deadline;
    assert(std::abs(expiry-10.05)<1e-10);
    assert(a.release_fresh(expiry,.4)); assert(!a.release_fresh(expiry+1e-8,.4));
    assert(!a.current_fresh(10.051));
    a.joy(100000000000LL,0,.8,true,true,100.19,10.04);
    assert(a.drain(10.04).empty()); assert(!a.current_fresh(10.051));
    assert(a.deadline(100,100.200001,10)<0);
    std::cout<<"PASS remaining source lifetime, exact deadline, duplicates never extend expiry\n";
  }
  {
    InputAssembler a(.2,.2); initial_state(a);
    a.pose(100000000000LL,p,100,10);
    for(int k=0;k<24;++k) {
      double dt=k*.004;
      a.pose(100000000000LL+(k+1)*4000000LL,p,100+dt,10+dt);
      a.joy(100000000000LL+k*4000000LL,0,.8,true,true,100+dt,10+dt);
      auto f=a.drain(10+dt); assert(f.size()==1 && f[0].grip_value==0);
      assert(a.release_fresh(10+dt,.4));
    }
    assert(a.pending_size()<=1);
    std::cout<<"PASS Pose(k+1)/Joy(k) consumes every complete release without starvation\n";
  }
  {
    InputAssembler a(.2,.2); initial_state(a);
    a.pose(100000000000LL,p,100,10);
    a.pose(100008000000LL,p,100.008,10.008);
    a.joy(100008000000LL,1,0,true,true,100.008,10.008);
    assert(a.drain(10.008).size()==1);
    assert(!a.take_interruption());
    a.joy(100000000000LL,0,1,false,false,100.3,10.009);
    assert(a.current_fresh(10.009) && !a.take_interruption());
    a.joy(100000000000LL,0,1,true,true,100.016,10.016);
    assert(a.drain(10.016).empty());
    a.pose(100020000000LL,p,100.02,10.02);
    a.tracking_state(-100021000000LL,10.021); assert(a.take_interruption());
    a.tracking_state(100024000000LL,10.024);
    a.joy(100020000000LL,0,1,true,true,100.024,10.024);
    assert(a.drain(10.024).empty()); // cannot complete a pre-restart partial
    a.pose(100028000000LL,p,100.028,10.028);
    a.joy(100028000000LL,1,.8,true,true,100.028,10.028);
    assert(a.drain(10.028).size()==1 && !a.release_fresh(10.028,.4));
    std::cout<<"PASS dropped/out-of-order frames and source-restart epoch barrier\n";
  }
  {
    InputAssembler a(.2,.2);initial_state(a);
    ControllerPose older,newer;older.tran.x=1;newer.tran.x=2;
    a.pose(100008000000LL,newer,100.009,10.000);
    a.pose(100000000000LL,older,100.01,10.001);
    a.joy(100000000000LL,0,0,true,true,100.011,10.002);
    a.joy(100008000000LL,1,0,true,true,100.012,10.003);
    auto f=a.drain(10.004);assert(f.size()==2);
    assert(f[0].stamp_ns<f[1].stamp_ns && f[0].pose_receive>f[1].pose_receive);
    std::cout<<"PASS complete source ordering remains distinct from reversed callback receive ordering\n";
  }
  {
    InputAssembler a(.2,.2,true,4); initial_state(a);
    for(int k=0;k<100;++k) a.pose(100000000000LL+k*1000,p,100,10);
    assert(a.pending_size()==4 && a.dropped()==96 && a.take_interruption());
    for(int k=100;k<120;++k) {
      a.pose(100000000000LL+k*1000,p,100,10);
      a.joy(100000000000LL+k*1000,0,0,true,true,100,10);
    }
    assert(a.ready_size()==4 && a.pending_size()<=4);
    std::cout<<"PASS partial/complete buffers bounded with counted overflow interruption\n";
  }
  for(double value:{-.01,1.01,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    InputAssembler a(.2,.2); initial_state(a);
    a.pose(100000000000LL,p,100,10); a.joy(100000000000LL,value,0,true,true,100,10);
    assert(a.drain(10).empty() && a.take_interruption());
    a.pose(100008000000LL,p,100.008,10.008); a.joy(100008000000LL,0,value,true,true,100.008,10.008);
    assert(a.drain(10.008).empty() && a.take_interruption());
  }
  {
    InputAssembler a(.2,.2); initial_state(a);
    a.pose(100000000000LL,p,100,10); a.joy(100000000000LL,0,1,false,true,100,10);
    assert(a.drain(10).empty() && !a.release_fresh(10,.4));
    a.pose(100008000000LL,p,100.008,10.008); a.joy(100008000000LL,1,.8,true,false,100.008,10.008);
    assert(a.drain(10.008).empty());
    std::cout<<"PASS invalid/missing Grip and Trigger metadata never create release\n";
  }
  {
    InputAssembler a(.2,.2);initial_state(a);
    a.pose(100000000000LL,p,100,10);a.joy(100000000000LL,1,0,true,true,100,10);a.drain(10);
    assert(a.tracking_state(-100032000000LL,10.04));
    assert(a.tracking_state(100040000000LL,10.05));
    // Entire release/press/move frames were generated before loss, and neither
    // half was seen then. Each topic is FIFO, all messages are delivered.
    for(int i=1;i<=3;++i) {
      ControllerPose old;old.tran.x=i==3 ? 200 : 0;
      const int64_t stamp=100000000000LL+i*8000000LL;
      a.pose(stamp,old,100.051+i*.001,10.051+i*.001);
      a.joy(stamp,i==1 ? 0 : 1,.8,true,true,100.051+i*.001,10.051+i*.001);
      assert(a.drain(10.051+i*.001).empty());
      assert(!a.release_fresh(10.051+i*.001,.4));
    }
    a.joy(100032000000LL,0,.8,false,false,100.056,10.056);
    a.pose(100048000000LL,p,100.057,10.057);a.joy(100048000000LL,1,.8,true,true,100.057,10.057);
    assert(a.drain(10.057).size()==1 && !a.release_fresh(10.057,.4));
    a.pose(100064000000LL,p,100.064,10.064);a.joy(100064000000LL,0,.8,true,true,100.064,10.064);
    assert(a.drain(10.064).size()==1 && a.release_fresh(10.064,.4));
    a.pose(100072000000LL,p,100.072,10.072);a.joy(100072000000LL,1,.8,true,true,100.072,10.072);
    assert(a.drain(10.072).size()==1 && !a.release_fresh(10.072,.4));
    std::cout<<"PASS source recovery barrier rejects whole unseen in-flight frames, held recovery waits and fresh release works\n";
  }
  {
    InputAssembler a(.2,.2);initial_state(a);
    // Even if the negative transition is lost, the new positive heartbeat
    // fences the unseen pre-loss generation.
    assert(a.tracking_state(100040000000LL,10.05));
    a.pose(100008000000LL,p,100.051,10.051);a.joy(100008000000LL,0,.8,true,true,100.051,10.051);
    assert(a.drain(10.051).empty());
    a.pose(100048000000LL,p,100.052,10.052);a.joy(100048000000LL,1,.8,true,true,100.052,10.052);a.drain(10.052);
    assert(!a.tracking_state(-100030000000LL,10.053)); // backward event clock: fail closed
    assert(!a.current_fresh(10.053));
    assert(!a.tracking_state(100040000000LL,10.054)); // same epoch cannot repair a clock conflict
    assert(a.tracking_state(100064000000LL,10.064));
    assert(a.tracking_state(100064000000LL,10.065)); // healthy fixed barrier heartbeat refreshes age
    std::cout<<"PASS repeated recovery marker covers lost transition; clock rollback requires a newer epoch\n";
  }
  {
    InputAssembler a(.2,.2);initial_state(a);
    a.pose(100000000000LL,p,100,10);a.joy(100000000000LL,0,.8,true,true,100,10);a.drain(10);
    assert(a.trigger_request_current(100000000000LL,.8,.02,10));
    a.joy(100008000000LL,0,0,true,true,100.008,10.008); // no corresponding Pose yet
    assert(!a.trigger_request_current(100000000000LL,.8,.02,10.008));
    a.joy(100016000000LL,0,.8,true,true,100.016,10.016); // a change back cannot revive old authorization
    assert(!a.trigger_request_current(100000000000LL,.8,.02,10.016));
    a.pose(100016000000LL,p,100.017,10.017);assert(a.drain(10.017).size()==1);
    assert(a.trigger_request_current(100016000000LL,.8,.02,10.017));
    a.joy(100008000000LL,0,0,true,true,100.018,10.018);
    assert(a.trigger_request_current(100016000000LL,.8,.02,10.018));
    std::cout<<"PASS unpaired Joy withdrawal revokes immediately, change-back needs new pairing, old replay cannot cancel\n";
  }
  {
    InputAssembler a(.2,.2,false); // opt-out only affects heartbeat TTL, never epoch fencing
    a.pose(100000000000LL,p,100,10);a.joy(100000000000LL,0,.8,true,true,100,10);
    assert(a.drain(10).empty());
    assert(a.tracking_state(100008000000LL,10.008));
    a.pose(100008000000LL,p,100.009,10.009);a.joy(100008000000LL,0,.8,true,true,100.009,10.009);
    assert(a.drain(10.009).size()==1);
    std::cout<<"PASS disabling tracking heartbeat timeout does not bypass recovery epoch\n";
  }

}
