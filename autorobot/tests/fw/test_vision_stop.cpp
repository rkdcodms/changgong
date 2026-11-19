#include "Arduino.h"
#include <sstream>
#include <tuple>
#include <vector>
#include "sources.h"
namespace Original {
#include ORIGINAL_SKETCH
}
namespace Vision {
#include VISION_SKETCH
}
Device board,baseline;
int checks=0;
const char* scenario="initial";
void expect(bool ok,const char* msg) {
  ++checks;
  if(!ok) { fprintf(stderr,"FAIL [%s t=%u]: %s\n",scenario,board.now,msg); exit(1); }
}
void mask(Device& d,int bits) { for(int i=0;i<8;++i)d.adc[i]=(bits&(1<<i))?50:788; }
#define RESET_COMMON(ns) \
  memset(ns::white,0,sizeof(ns::white)); memset(ns::sensorValues,0,sizeof(ns::sensorValues)); \
  ns::whiteCount=ns::allWhiteStreak=0; ns::startupChecked=false; ns::error=0; \
  ns::previousError=ns::filteredDerivative=0; ns::pdReady=false; ns::lastDirection=0; \
  ns::leftCommand=ns::rightCommand=0; ns::stateSince=ns::lastControl=ns::stableSince=0; \
  ns::stableActive=false; ns::lastSensorPrint=0; \
  ns::stopSequence=ns::lastStatePrint=0; ns::lastReportedMode=-1
#define CONTROL_STATE(ns) std::make_tuple(int(ns::mode),ns::whiteCount,ns::allWhiteStreak,ns::startupChecked, \
  ns::error,ns::previousError,ns::filteredDerivative,ns::pdReady,ns::lastDirection, \
  ns::leftCommand,ns::rightCommand,ns::stateSince,ns::lastControl,ns::stableSince,ns::stableActive)
void reset(const char* name,int bits=0x18,uint32_t at=0) {
  scenario=name; board=Device();board.now=at;mask(board,bits);activeDevice=&board;
  RESET_COMMON(Vision);Vision::mode=Vision::WAIT_VISION;
  Vision::visionSession=Vision::lastHeartbeat=Vision::lastAcceptedEvent=0;
  Vision::lastVisionClass=-1;Vision::stopReason=0;
  Vision::visionRxLength=0;Vision::visionRxDropping=false;Vision::visionRxAt=0;
  Vision::lastVisionPrint=Vision::reportedSession=Vision::reportedEvent=0;
  Vision::reportedClass=-2;Vision::reportedReason=Vision::reportedLive=-1;
  Vision::setup();
}
void feed(const std::string& line) {
  activeDevice=&board;board.rx+=line;
  while(Serial.available()) {board.room=63;Vision::loop();}
}
void tick(int bits=0x18,uint32_t dt=10,bool heartbeat=true,int room=63) {
  board.now+=dt;mask(board,bits);activeDevice=&board;board.room=room;
  if(heartbeat && Vision::visionSession) board.rx+="@VC1,H,"+std::to_string(Vision::visionSession)+"\n";
  Vision::loop();
}
bool braked() {
  return board.pwm[5]==255 && board.pwm[6]==255 &&
    board.level[30]==0 && board.level[31]==0 && board.level[32]==0 && board.level[33]==0 &&
    Vision::leftCommand==0 && Vision::rightCommand==0;
}
void arm() { feed("@VC1,H,42\n");expect(Vision::mode==Vision::FOLLOW,"heartbeat releases WAIT"); }
void stop(unsigned event=1,unsigned cls=3) {
  feed("@VC1,S,42,"+std::to_string(event)+","+std::to_string(cls)+"\n");
  expect(Vision::mode==Vision::STOPPED,"vision enters STOPPED");expect(braked(),"vision immediately brakes");
}
void finishStop(int bits) { for(int i=0;i<500;++i)tick(bits); }

int main() {
  reset("boot waits");
  for(int i=0;i<200;++i)tick(0x18,10,false);
  expect(Vision::mode==Vision::WAIT_VISION && braked(),"no heartbeat never moves");
  expect(board.tx.find("@V1,0,0,-1,0,0\n")!=std::string::npos,"initial V1 includes unclaimed session and class");
  expect(board.tx.find("@LT1,5,")!=std::string::npos,"WAIT is LT1 mode 5");

  reset("strict parser");
  const char* invalid[]={"@VC1,H,0\n","@VC1,H,-1\n","@VC1,H,+1\n","@VC1,H, 1\n","@VC1,H,4294967296\n","@VC1,H,00000000001\n","@VC1,H,\n","@VC1,H,1,2\n","@VC1,Q,1\n","@VC2,H,1\n","@VC1,H,1\r\n","@VC1,S,42,1,1\n"};
  for(auto line:invalid) {feed(line);expect(Vision::mode==Vision::WAIT_VISION && Vision::visionSession==0,"invalid frame cannot arm");}
  feed(std::string(100,'x')+"\n@VC1,H,42\n");expect(Vision::visionSession==42,"overflow resynchronizes at LF");
  feed("@VC1,S,42,1,8\n");feed("@VC1,S,42,1,-1\n");feed("@VC1,S,42,0,1\n");feed("@VC1,S,99,1,1\n");
  expect(Vision::mode==Vision::FOLLOW && Vision::lastAcceptedEvent==0,"invalid stop fields cannot stop");

  reset("partial frames");feed("@VC1,H,42");expect(Vision::visionSession==0,"partial frame never acts");
  feed("\n");expect(Vision::visionSession==42,"completed partial frame acts");
  reset("stale partial discarded");feed("@VC1,H,");tick(0x18,250,false);feed("@VC1,H,42\n");expect(Vision::visionSession==42,"stale partial does not block next full frame");

  reset("session and timeout");arm();tick(0x18,100,false);uint32_t hb=Vision::lastHeartbeat;
  feed("@VC1,H,99\n");expect(Vision::visionSession==42 && Vision::lastHeartbeat==hb,"foreign heartbeat ignored");
  tick(0x18,1399,false);expect(Vision::mode==Vision::FOLLOW,"1499ms heartbeat age still live");
  board.rx+="@VC1,H,42\n";tick(0x18,1,false);
  expect(Vision::mode==Vision::FAULT && Vision::stopReason==3 && braked(),"1500ms timeout latches before queued heartbeat");
  feed("@VC1,H,42\n@VC1,S,42,1,2\n");expect(Vision::mode==Vision::FAULT && braked(),"heartbeat and stop cannot revive FAULT");

  reset("five seconds duplicate and recovery");arm();tick();stop();const auto since=Vision::stateSince;
  for(int i=0;i<499;++i) {if(i%17==0)feed("@VC1,S,42,1,3\n@VC1,S,42,2,4\n");tick();}
  expect(Vision::mode==Vision::STOPPED && Vision::stateSince==since && Vision::lastAcceptedEvent==1 && braked(),"duplicate or new S during stop never extends or replaces it");
  tick();expect(board.now-since==5000 && Vision::mode==Vision::FOLLOW,"resume on normal line exactly at 5000ms");
  expect(Vision::stopReason==0,"resume clears active stop reason");
  feed("@VC1,S,42,1,3\n");expect(Vision::mode==Vision::FOLLOW,"accepted event replay does not stop after resume");
  stop(2,4);expect(Vision::lastAcceptedEvent==2 && Vision::lastVisionClass==4,"new event accepted after rearm");
  feed("@VC1,S,42,1,3\n");expect(Vision::lastAcceptedEvent==2,"older event ignored");

  reset("first control after H and S on all white",255);arm();stop();tick(255);
  expect(Vision::mode==Vision::STOPPED && braked(),"initial white never overrides early vision stop");
  for(int i=1;i<500;++i)tick(255);
  expect(Vision::mode==Vision::EXIT_BAR,"vision resume with all white uses EXIT_BAR");

  reset("resume searches lost line");arm();tick(0x03);stop();finishStop(0);
  expect(Vision::mode==Vision::SEARCH,"no white after stop resumes SEARCH");
  expect(Vision::lastDirection==1,"resume keeps previous known turn direction");
  for(int i=0;i<6;++i)tick(0);
  expect(Vision::leftCommand!=Vision::rightCommand,"known turn direction does not blindly drive straight");

  reset("white bar pass");arm();tick();tick(255);
  expect(Vision::mode==Vision::EXIT_BAR && Vision::stopSequence==0,"white bar passes without five-second stop");
  stop();expect(Vision::stopReason==2,"vision stop accepted during bar exit");

  reset("timeout during stop");arm();stop();tick(0x18,1500,false);
  expect(Vision::mode==Vision::FAULT && braked(),"link loss during stop latches FAULT");
  for(int i=0;i<600;++i)tick();expect(Vision::mode==Vision::FAULT && braked(),"heartbeat recovery and elapsed five seconds never restart FAULT");

  reset("rx budget");board.rx=std::string(1000,'x');board.room=63;size_t before=board.readCount;Vision::loop();
  expect(board.readCount-before==32,"one loop consumes no more than 32 serial bytes");

  reset("uint32 wrap",0x18,UINT32_MAX-500);arm();tick();stop();finishStop(0x18);
  expect(Vision::mode==Vision::FOLLOW,"five-second stop works across millis wrap");
  tick(0x18,1500,false);expect(Vision::mode==Vision::FAULT,"heartbeat timeout works after wrap");

  reset("maximum IDs and nonblocking telemetry");feed("@VC1,H,4294967295\n");feed("@VC1,S,4294967295,4294967295,7\n");
  expect(Vision::lastAcceptedEvent==UINT32_MAX && Vision::lastVisionClass==7,"maximum uint32 IDs accepted");
  for(int i=0;i<100;++i)tick(0x18,10,true,(i%4==0)?0:63);
  expect(board.tx.find("@V1,4294967295,4294967295,7,2,1\n")!=std::string::npos,"V1 ack survives constrained TX capacity");
  std::istringstream rows(board.tx);std::string row;int v1=0,lt1=0;
  while(std::getline(rows,row)) if(row.rfind("@V1,",0)==0 || row.rfind("@LT1,",0)==0) {
    expect(row.size()+1<=63,"telemetry fits hardware TX buffer");
    if(row.rfind("@V1,",0)==0)++v1;else ++lt1;
  }
  expect(v1>=8 && lt1>=8,"both telemetry streams continue under backpressure");
  expect(board.blockingPrints==0,"new steady-state telemetry never blocks");

  reset("normal control equivalence");arm();baseline=Device();mask(baseline,0x18);activeDevice=&baseline;
  RESET_COMMON(Original);Original::mode=Original::FOLLOW;Original::setup();
  for(int i=0;i<1500;++i) {
    int bits=(i%200<50)?0x18:((i%200<100)?0x03:((i%200<115)?0:0x30));
    baseline.now+=10;mask(baseline,bits);baseline.room=63;activeDevice=&baseline;Original::loop();
    tick(bits);
    expect(CONTROL_STATE(Original)==CONTROL_STATE(Vision),"normal line/search control state matches original");
    expect(memcmp(baseline.pwm,board.pwm,sizeof(board.pwm))==0,"normal motor PWM matches original");
    expect(memcmp(baseline.level,board.level,sizeof(board.level))==0,"normal motor directions match original");
  }
  printf("PASS: %d assertions; parser, interlocks, five-second stop, recovery, telemetry, original control equivalence\n",checks);
}
