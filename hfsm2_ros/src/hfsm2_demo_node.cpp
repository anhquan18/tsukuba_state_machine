// Minimal HFSM2 state machine that mirrors Nav2's conceptual flow.
// States print on entry; transitions are driven by events.

#include <iostream>

#define HFSM2_ENABLE_PLANS
#include <hfsm2/machine.hpp>

/**
 * Context
 * Context will hold data shared across between states.
 */
struct Context {};

/**
 * Define type config
 */
using Config = hfsm2::Config::ContextT<Context>;
using M = hfsm2::MachineT<Config>;

//---------------- Events to drive transitions
struct Goal        {};  // request navigation
struct PlanOk      {};  // global plan computed
struct PlanFail    {};  // planning failed
struct ControlOk   {};  // controller finished
struct ControlFail {};  // controller failed
struct RecoveryOk  {};  // recovery finished
struct RecoveryFail{};  // recovery failed
struct Cancel      {};  // user canceled

//---------------- Forward declare states
struct Idle;
struct Planning;
struct Controlling;
struct Recovery;
struct Succeeded;
struct Failed;
struct BatteryReplace;

// States need to be forward declared
#define S(s) struct s
//---------------- FSM layout (single active region)
using FSM = M::PeerRoot<Idle, Planning, Controlling, Recovery, Succeeded,Failed>;
#undef S

// double-check state ids for Logger::stateName()
static_assert(FSM::stateId<Idle>()	  ==  1, "");
static_assert(FSM::stateId<Planning>() ==  2, "");
static_assert(FSM::stateId<Controlling>()  ==  3, "");
static_assert(FSM::stateId<Recovery>()  ==  4, "");
static_assert(FSM::stateId<Succeeded>()  ==  5, "");
static_assert(FSM::stateId<Failed>()  ==  6, "");

//---------------- Helper: print on state entry
static void say(const char* text) {
  std::cout << text << std::endl;
}

//---------------- States (method shapes per HFSM2 manual)
struct Idle : FSM::State {
  using FSM::State::react;
  void enter(PlanControl&)                    { say("→ Idle"); }
  void update(FullControl & control) {}
  void exit(PlanControl & control) {}
  void react(const Goal&, EventControl& ctl)  { ctl.changeTo<Planning>(); }
};

struct Planning : FSM::State {
  using FSM::State::react;
  void enter(PlanControl&)                       { say("→ Planning (compute global path)"); }
  void update(FullControl & control) {}
  void exit(PlanControl & control) {}
  void react(const PlanOk&,   EventControl& ctl) { ctl.changeTo<Controlling>(); }
  void react(const PlanFail&, EventControl& ctl) { ctl.changeTo<Recovery>();    }
  void react(const Cancel&,   EventControl& ctl) { ctl.changeTo<Failed>();      }
};

struct Controlling : FSM::State {
  using FSM::State::react;
  void enter(PlanControl&)                          { say("→ Controlling (follow path)"); }
  void update(FullControl & control) {}
  void exit(PlanControl & control) {}
  void react(const ControlOk&,   EventControl& ctl) { ctl.changeTo<Succeeded>(); }
  void react(const ControlFail&, EventControl& ctl) { ctl.changeTo<Recovery>();  }
  void react(const Cancel&,      EventControl& ctl) { ctl.changeTo<Failed>();    }
};

struct Recovery : FSM::State {
  using FSM::State::react;
  void enter(PlanControl&)                            { say("→ Recovery (clear/spin/backup)"); }
  void update(FullControl & control) {}
  void exit(PlanControl & control) {}
  void react(const RecoveryOk&,   EventControl& ctl)  { ctl.changeTo<Planning>(); }
  void react(const RecoveryFail&, EventControl& ctl)  { ctl.changeTo<Failed>();   }
};

struct Succeeded : FSM::State {
  void enter(PlanControl&) { say("✅ Succeeded"); }
  void update(FullControl & control) {}
  void exit(PlanControl & control) {}
};

struct Failed : FSM::State {
  void enter(PlanControl&) { say("❌ Failed"); }
  void update(FullControl & control) {}
  void exit(PlanControl & control) {}
};

struct BatteryReplace : FSM::State {
  void enter(PlanControl&) { say("Battery Replace Mode"); }
  void update(FullControl & control) {}
  void exit(PlanControl & control) {}
};

//---------------- Demo driver
int main() {
  Context ctx{};
  FSM::Instance fsm{ctx}; // enters Idle

  // Example sequence (feel free to edit):
  fsm.react(Goal{});         // Idle -> Planning
  fsm.react(PlanOk{});       // Planning -> Controlling
  fsm.react(ControlFail{});  // Controlling -> Recovery
  fsm.react(RecoveryOk{});   // Recovery -> Planning
  fsm.react(PlanOk{});       // Planning -> Controlling
  fsm.react(ControlOk{});    // Controlling -> Succeeded

  return 0;
}
