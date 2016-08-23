
#include "SwitchStateMachine.h"

  void SwitchStateMachine::switchState(uint8_t oldstate,uint8_t newstate,uint8_t dly) {
    DPRINT(F("Switch State: "));
    DHEX(newstate);
    DPRINT(F(" ["));
    DHEX(dly);
    DPRINTLN(F("]"));
  }

  void SwitchStateMachine::jumpToTarget(SwitchPeerList lst) {
    uint8_t next = getNextState(state,lst);
    if( next != AS_CM_JT_NONE ) {
      // first cancel possible running alarm
      aclock.cancel(alarm);
      // get delay
      uint8_t dly = getDelayForState(next,lst);
      // signal state change, if any
      if( state != next ) {
        switchState(state,next,dly);
        state = next;
      }
      if( dly == 0 ) {
        // switch immediately to next state
        jumpToTarget(lst);
      }
      else if( dly != 0xff ) {
        // setup alarm to process after delay
        alarm.list(lst);
        alarm.set(byteTimeCvt(dly));
        aclock.add(alarm);
      }
    }
  }

  uint8_t SwitchStateMachine::getNextState(uint8_t state,SwitchPeerList lst) {
    switch( state ) {
      case AS_CM_JT_ONDELAY:  return lst.jtDlyOn();
      case AS_CM_JT_ON:       return lst.jtOn();
      case AS_CM_JT_OFFDELAY: return lst.jtDlyOff();
      case AS_CM_JT_OFF:      return lst.jtOff();
    }
    return AS_CM_JT_NONE;
  }

  uint8_t SwitchStateMachine::getDelayForState(uint8_t state,SwitchPeerList lst) {
    switch( state ) {
      case AS_CM_JT_ONDELAY:  return lst.onDly();
      case AS_CM_JT_ON:       return lst.onTime();
      case AS_CM_JT_OFFDELAY: return lst.offDly();
      case AS_CM_JT_OFF:      return lst.offTime();
      default: break;
    }
    return 0;
  }

