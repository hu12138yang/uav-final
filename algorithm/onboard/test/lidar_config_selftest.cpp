#include <onboard/config_file.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cstdio>
#include <unistd.h>
int main(int argc,char **argv) {
  if(argc!=2)return 1;ego_onboard::AppConfig config;std::string error;
  if(!ego_onboard::loadConfigFile(argv[1],config,error)||!config.lidar.allow_zero_crc||config.lidar.expected_serial!="ARMCP8H0037373"||config.lidar.point_data_type!=1||config.lidar.frame_duration_s!=.1)return 1;
  char path[]="/tmp/uav-lidar-config-XXXXXX";int fd=mkstemp(path);if(fd<0)return 1;close(fd);
  bool ok=true;
  auto check=[&](YAML::Node root,bool expected){std::ofstream out(path);out<<root;out.close();ego_onboard::AppConfig parsed;std::string err;const bool result=ego_onboard::loadConfigFile(path,parsed,err);ok=ok&&(result==expected);};
  auto root=YAML::LoadFile(argv[1]);root["lidar"]["crc_policy"]="strict";root["lidar"]["expected_serial"]="";check(root,true);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=0.5;check(root,true);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=0;check(root,false);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=2.1;check(root,false);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=".nan";check(root,false);
  root=YAML::LoadFile(argv[1]);root["map"]["obstacles_inflation"]=1.5;check(root,false);
  root=YAML::LoadFile(argv[1]);root["vehicle"]["min_obstacle_gap"]=".nan";check(root,false);
  root=YAML::LoadFile(argv[1]);root["vehicle"]["radius"]=1.4;check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["expected_serial"]="";check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["crc_policy"]="anything";check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["queue_capacity"]=-1;check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["frame_duration_s"]=".nan";check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["min_range_m"]=100;check(root,false);
  unlink(path);std::printf("Measured lidar profile config and invalid settings rejected=%d\n",ok);return ok?0:1;
}
