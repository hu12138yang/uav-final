#include <onboard/mission_input.h>
#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace ego_onboard;
namespace {
const char *valid = "<kml><Placemark><LineString><coordinates>113,21,0 113.001,21,0</coordinates></LineString></Placemark></kml>";
void write(const std::string &path, const std::string &text) { std::ofstream out(path); out << text; }
}
int main() {
  int failures=0;
  auto check=[&](bool ok,const char *name){std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);if(!ok)++failures;};
  char directory[]="/tmp/uav-mission-inbox-XXXXXX";
  if(!mkdtemp(directory)) return 2;
  const std::string path=std::string(directory)+"/mission.kml", temporary=path+".tmp";
  MissionInputConfig cfg;cfg.file=path;cfg.wait_for_file=true;
  MissionInput input;input.start(cfg);
  input.step(1);input.step(1000);
  check(input.state()==MissionInputState::WAIT_KML&&input.mission().route.empty(),"missing KML waits indefinitely without a task");
  write(path,"<kml>");input.step(1001);input.step(1001.4);
  check(input.state()==MissionInputState::WAIT_KML,"a new file must settle before it is read");
  input.step(1001.6);input.step(1001.6);input.step(1001.6);
  check(input.state()==MissionInputState::WAIT_KML&&!input.error().empty()&&input.mission().route.empty(),"partial/invalid KML is rejected while the inbox remains active");
  input.step(1010);
  check(input.state()==MissionInputState::WAIT_KML,"unchanged rejected content is not retried as a new task");
  write(temporary,valid);std::rename(temporary.c_str(),path.c_str());
  input.step(1011);input.step(1011.6);input.step(1011.6);input.step(1011.6);input.step(1011.6);
  check(input.state()==MissionInputState::READY&&input.mission().route.size()==3,"atomic valid delivery starts one closed task automatically");
  const auto saved=input.mission().route;
  write(path,"broken replacement");input.step(1012);
  check(input.state()==MissionInputState::READY&&input.mission().route.size()==saved.size()&&input.mission().route[1].lon_deg==saved[1].lon_deg,"READY latches the task; replacement cannot change the active route");
  write(path,valid);input.start(cfg);input.step(2000);input.step(2001);input.step(2001);
  check(input.state()==MissionInputState::PARSING_KML,"load a stable file for concurrent replacement check");
  write(temporary,"invalid new delivery");std::rename(temporary.c_str(),path.c_str());
  input.step(2001);input.step(2001);
  check(input.state()==MissionInputState::WAIT_KML&&input.mission().route.empty(),"file changed during validation cannot activate the old contents");
  std::remove(path.c_str());input.start(cfg);input.step(3000);
  check(input.state()==MissionInputState::WAIT_KML&&input.mission().route.empty(),"restart clears any previous prepared task");
  cfg.wait_for_file=false;input.start(cfg);input.step();
  check(input.state()==MissionInputState::FAILED,"strict prepare-only mode still fails immediately on a missing file");
  cfg.wait_for_file=true;cfg.file_settle_s=-1;input.start(cfg);
  check(input.state()==MissionInputState::FAILED,"invalid settle configuration is not silently accepted");
  rmdir(directory);
  return failures?1:0;
}
