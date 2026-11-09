// Local KML -> FCU mission -> GCS takeoff -> synchronized LiDAR/EGO -> return/hold.
#include <onboard/async_planner.h>
#include <onboard/config_file.h>
#include <onboard/kml_planner_start.h>
#include <onboard/livox_source.h>
#include <onboard/route_utils.h>
#include "serial_port.h"
#include "status_uplink.h"
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sys/stat.h>
using namespace ego_onboard;
using namespace ego_mav;
namespace {
volatile std::sig_atomic_t stopped=0;
void stop(int) { stopped=1; }
void usage() {
  std::puts("onboard_main --lidar-only --config <yaml> --livox-config <sdk.json> --lidar-ip <ip> [--duration <seconds>]\n"
    "onboard_main --prepare-only --config <yaml> --kml <path>\n"
    "onboard_main --serial <device|tcp://host:port> --config <yaml> [--baud 921600]\n"
    "             [--livox-config <sdk.json> --lidar-ip <ip>] [--record-dir <directory>]\n"
    "             [--duration <seconds>] [--enable-control --kml <path>] [--uplink <ip:port>]\n"
    "             [--no-wait-kml] (observe-only diagnostics; skip the default inbox)\n"
    "             [--diagnose-zero-point-crc] (observe-only raw inspection; no trusted mapping)\n"
    "Waits for one stable valid KML at the configured path, then starts automatically.\n"
    "No arm/takeoff commands; GCS starts flight. --duration begins after KML loading.\n"
    "Physical mode defaults to observe-only. No synthetic cloud is permitted.\n"
    "Observe-only sends heartbeat, telemetry requests and clock sync only.");
}
void snapshot(const std::string &path,const ego_sensor::LidarFrame &f) {
  std::ofstream out(path);
  out << "# Raw sensor coordinates; NOT deskewed\nVERSION .7\nFIELDS x y z intensity time\nSIZE 4 4 4 4 8\nTYPE F F F F F\nCOUNT 1 1 1 1 1\nWIDTH " << f.points.size()
      << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << f.points.size() << "\nDATA ascii\n";
  for(const auto &p:f.points) out << p.p.x()<<' '<<p.p.y()<<' '<<p.p.z()<<' '<<p.reflectivity<<' '<<std::setprecision(12)<<p.t_rel<<'\n';
}
}
int main(int argc,char **argv) {
  bool prepare=false,control=false,diagnose_zero_crc=false,lidar_only=false,no_wait_kml=false,kml_override=false;
  std::string dev,config,kml,lidar_config,lidar_ip,record,uplink_override;
  int baud=921600; double duration=0;
  try {
    for(int i=1;i<argc;++i) {
      const std::string a=argv[i];
      auto value=[&]() -> std::string { if(i+1>=argc) throw std::runtime_error("missing value for "+a); return argv[++i]; };
      if(a=="--lidar-only") lidar_only=true;
      else if(a=="--serial") dev=value();
      else if(a=="--config") config=value();
      else if(a=="--kml") {kml=value();kml_override=true;}
      else if(a=="--livox-config") lidar_config=value();
      else if(a=="--lidar-ip") lidar_ip=value();
      else if(a=="--record-dir") record=value();
      else if(a=="--uplink") uplink_override=value();
      else if(a=="--baud") baud=std::stoi(value());
      else if(a=="--duration") duration=std::stod(value());
      else if(a=="--prepare-only") prepare=true;
      else if(a=="--no-wait-kml") no_wait_kml=true;
      else if(a=="--enable-control") control=true;
      else if(a=="--observe-only") control=false;
      else if(a=="--diagnose-zero-point-crc") diagnose_zero_crc=true;
      else if(a=="--help" || a=="-h") {usage();return 0;}
      else throw std::runtime_error("unknown option: "+a);
    }
    if(!std::isfinite(duration)||duration<0) throw std::runtime_error("duration must be finite and nonnegative");
    if(!prepare&&!lidar_only&&dev.empty()) throw std::runtime_error("--serial is required unless using --prepare-only or --lidar-only");
    if(diagnose_zero_crc && (control || prepare)) throw std::runtime_error("zero CRC diagnostics require physical observe-only mode");
    if(lidar_only&&(prepare||control||!dev.empty()||lidar_config.empty()||lidar_ip.empty())) throw std::runtime_error("lidar-only requires SDK config and lidar IP, without serial/prepare/control");
    if(config.empty()) throw std::runtime_error("--config is required to preserve deployment parameters");
    if(no_wait_kml&&control) throw std::runtime_error("--no-wait-kml is only for observe-only diagnostics");
    if(lidar_only&&kml_override) throw std::runtime_error("--lidar-only does not load a KML");
    AppConfig app; std::string error;
    if(!loadConfigFile(config,app,error)) throw std::runtime_error(error);
    if(kml_override) app.mission_input.file=kml;
    if(prepare||no_wait_kml) app.mission_input.wait_for_file=false;
    if(lidar_only||(no_wait_kml&&!kml_override)) app.mission_input.file.clear();
    if(!uplink_override.empty()) app.uplink=uplink_override;
    std::vector<GeoWaypoint> route;
    std::signal(SIGINT,stop); std::signal(SIGTERM,stop); std::signal(SIGPIPE,SIG_IGN);
    // Always validate a supplied KML before opening any device.
    if(!app.mission_input.file.empty()) {
      MissionInput input; input.start(app.mission_input);
      if(app.mission_input.wait_for_file) {
        std::printf("[STATE] WAIT_KML path=%s; waiting for one stable valid task\n",app.mission_input.file.c_str());
        std::fflush(stdout);
      }
      std::string last_error;
      while(input.state()!=MissionInputState::READY && input.state()!=MissionInputState::FAILED && !stopped) {
        input.step();
        if(input.error()!=last_error) {
          last_error=input.error();
          if(!last_error.empty()) {std::printf("[KML] %s\n",last_error.c_str());std::fflush(stdout);}
        }
        if(input.state()==MissionInputState::WAIT_KML) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if(stopped) {std::puts("[STOP] KML wait cancelled; no device opened");return 0;}
      if(input.state()==MissionInputState::FAILED) throw std::runtime_error(input.error());
      route=input.mission().route;
      std::puts("[KML READY] valid task accepted; starting configured mode automatically");
      for(const auto &w:input.mission().warnings) std::printf("[KML] %s\n",w.c_str());
    }
    else if(control||prepare) throw std::runtime_error("physical control requires an explicit local KML");
    if(!route.empty()) {
      bool added=false;
      if(!ensureReturnToStart(route,app.mission_input.max_waypoints,added,error)) throw std::runtime_error(error);
      app.pilot.cruise_alt_m=route.front().alt_rel_m;
      std::printf("[KML] %zu reference points, %zu mission items, height %.2fm\n",route.size(),route.size(),route.front().alt_rel_m);
    }
    const bool kml_ready=route.size()>=2;
    if(prepare) { std::puts("[READY] KML validated; no device opened"); return 0; }
    if(control&&(lidar_config.empty()||lidar_ip.empty()||!app.extrinsic_confirmed))
      throw std::runtime_error("control requires --livox-config, --lidar-ip, and calibrated extrinsic.confirmed=true");
    if(control&&(app.pilot.gps_min_fix<6||!app.pilot.gps_failsafe||!app.pilot.sensing_failsafe||!app.pilot.use_planner))
      throw std::runtime_error("physical RTK control requires min_fix=6, GPS/sensing failsafes, and EGO planner");
    app.pilot.require_timesync=true;
    app.pilot.observe_only=!control;
    app.adapter.strict_pose_coverage=true;
    std::ofstream diag,frame_log,raw_rx,raw_tx,rx_chunks;
    uint64_t rx_offset=0;
    if(!record.empty()) {
      if(::mkdir(record.c_str(),0755)!=0&&errno!=EEXIST) throw std::runtime_error("cannot create record directory");
      if(!control&&!lidar_only) { raw_rx.open(record+"/fcu_rx.bin",std::ios::binary); raw_tx.open(record+"/fcu_tx.bin",std::ios::binary); rx_chunks.open(record+"/fcu_rx_chunks.csv"); rx_chunks<<"host_monotonic,offset,size\n"; }
      if(!control&&!lidar_only&&(!raw_rx||!raw_tx||!rx_chunks)) throw std::runtime_error("cannot open raw serial diagnostic logs");
      diag.open(record+"/timing.csv"); frame_log.open(record+"/scans.csv");
      if(!diag||!frame_log) throw std::runtime_error("cannot open diagnostic logs");
      diag<<"elapsed,phase,gps_fix,clock_ok,sync_samples,rtt_ms,residual_ms,lidar_packets,time_type,aligned_frames,rejected_frames,map_frames,pose_fresh,max_loop_ms,crc_errors,ptp_ready,lidar_sync_offset_ns,lidar_crc_missing,kml_ready,planner_steps\n";
      frame_log<<"raw_epoch,host_monotonic,receive_monotonic,time_type,points,span_s,aligned,timing_ready,crc_verified,packet_policy_ok\n";
    }
    std::unique_ptr<AsyncPlanner> async(new AsyncPlanner(app.adapter,app.pilot.cruise_alt_m));
    PlannerIO *planner=async.get();
    OffboardPilot pilot; pilot.setConfig(app.pilot);pilot.setPlanner(planner);
    SerialPort serial; LivoxSource lidar;
    if(!lidar_only&&!serial.open(dev,baud,error)) throw std::runtime_error(error);
    if(!lidar_config.empty()&&!lidar.start(lidar_config,lidar_ip,error,diagnose_zero_crc,&app.lidar)) throw std::runtime_error(error);
    StatusUplink uplink;
    if(!app.uplink.empty()&&!uplink.open(app.uplink,error)) throw std::runtime_error(error);
    if(app.pilot.observe_only) pilot.startObserving(); else pilot.start(route);
    if(kml_ready) std::puts("[PLANNER] valid KML received: planner enabled; waiting for valid inputs");
    std::puts(lidar_only?"[LIDAR ONLY] Ethernet SDK2 directly to onboard; FCU serial is not opened":"[HARDWARE] real FCU; SDK2 cloud only");
    std::printf("[LIDAR PROFILE] policy=%s SN=%s type=%d frame=%.3fs range=[%.2f,%.2f] queue=%zu\n",app.lidar.allow_zero_crc?"mid360s_sample":"strict",app.lidar.expected_serial.c_str(),app.lidar.point_data_type,app.lidar.frame_duration_s,app.lidar.min_range_m,app.lidar.max_range_m,app.lidar.queue_capacity);
    std::puts(app.pilot.observe_only?"[OBSERVE] no task/mode/motion commands":"[CONTROL] GCS owns arm/takeoff/landing; onboard uploads ordinary waypoints");
    const double begin=monotonicSeconds(); double now=begin,previous=now,last_report=-1e9;
    double last_uplink=-1e9; uint64_t uplink_failures=0;
    bool ptp_ready=false;
    double max_loop=0,last_input_ok=-1e9,clock_stable_since=begin; uint64_t aligned=0,rejected=0;
    bool planner_bound=false;
    bool snapshot_done=false; Phase last_phase=pilot.phase();
    EpochBridge epoch; epoch.sample(monotonicSeconds(),unixSeconds());
    uint64_t sync_generation=0; bool failed=false;
    while(!stopped && (duration==0 || (now-begin)<duration)) {
      now=monotonicSeconds();
      const double gap=now-previous;previous=now;max_loop=std::max(max_loop,gap);
      if(control&&gap>0.15) {pilot.stop("control loop deadline exceeded");failed=true;break;}
      pilot.setReceiveTime(now);
      if(!lidar_only&&!serial.isOpen()) {pilot.stop("serial disconnected");failed=true;break;}
      std::vector<uint8_t> bytes;serial.take(bytes);
      if(!bytes.empty()) { if(raw_rx) { raw_rx.write(reinterpret_cast<const char*>(bytes.data()),bytes.size()); rx_chunks<<std::setprecision(16)<<now<<','<<rx_offset<<','<<bytes.size()<<'\n'; rx_offset+=bytes.size(); } pilot.onMavlinkBytes(bytes.data(),bytes.size()); }
      const bool stable=epoch.sample(monotonicSeconds(),unixSeconds());
      if(!stable || sync_generation!=pilot.timeAlignment().generation()) {
        clock_stable_since=now;last_input_ok=-1e9;sync_generation=pilot.timeAlignment().generation();
        // Mixed-clock maps must never be reused after a clock discontinuity.
        const bool active=pilot.phase()==Phase::PRE_OFFBOARD||pilot.phase()==Phase::OFFBOARD_CRUISE||pilot.phase()==Phase::HOLD;
        if(control&&active) { pilot.stop("clock discontinuity: restart perception before control");failed=true;break; }
        async.reset(new AsyncPlanner(app.adapter,app.pilot.cruise_alt_m)); planner=async.get(); pilot.setPlanner(planner);planner_bound=false;
      }
      const auto lidar_status=lidar.stats();
      const double last_sync=epoch.toMonotonic(lidar_status.last_sync_ns*1e-9);
      ptp_ready=lidar_status.sync_type==1 && now-lidar_status.state_rx<2.0 &&
                now-last_sync>=-0.005 && now-last_sync<2.0;
      ego_sensor::LidarFrame frame;
      while(lidar.pop(frame)) {
        const double raw=frame.t_base;
        frame.t_base=epoch.toMonotonic(raw);
        const double end=frame.points.empty()?frame.t_base:frame.t_base+frame.points.back().t_rel;
        const bool timing_ready=ptp_ready&&stable&&now-clock_stable_since>=2&&frame.time_type==1&&!frame.points.empty()&&
            pilot.clockReady(now)&&pilot.poseFresh(now)&&now-end<app.adapter.sensing_timeout_s&&end<=now+0.005;
        // Missing CRC and configured packet acceptance are separate properties.
        const bool crc_verified=frame.payload_crc_present;
        const bool packet_policy_ok=crc_verified||app.lidar.allow_zero_crc;
        const bool ok=!lidar_only&&!diagnose_zero_crc&&packet_policy_ok&&timing_ready;
        if(frame_log) frame_log<<std::setprecision(16)<<raw<<','<<frame.t_base<<','<<now<<','<<int(frame.time_type)<<','<<frame.points.size()<<','<<end-frame.t_base<<','<<ok<<','<<timing_ready<<','<<crc_verified<<','<<packet_policy_ok<<'\n';
        if(!control&&!snapshot_done&&!record.empty()) {snapshot(record+"/raw_cloud.pcd",frame);snapshot_done=true;}
        if(ok) {planner->onScanFrame(frame);last_input_ok=now;++aligned;} else ++rejected;
      }
      pilot.setInputReady(ptp_ready&&now-last_input_ok<app.adapter.sensing_timeout_s&&now-lidar.stats().last_rx<app.adapter.sensing_timeout_s);
      if(kml_ready&&!planner_bound&&planner&&pilot.origin().valid)
        planner_bound=bindKmlPlanner(route,pilot.origin(),*planner);
      // A valid KML begins onboard planning before arm/takeover. Preview is not sent.
      if(!lidar_only&&planner) {
        const bool inputs_ready=pilot.poseFresh(now)&&pilot.clockReady(now)&&
          pilot.gpsFixType()>=app.pilot.gps_min_fix&&planner->sensingHealthy();
        preplanKml(kml_ready,planner_bound,inputs_ready,pilot.phase(),now,pilot.latestPose().p,*planner);
      }
      std::vector<std::vector<uint8_t>> tx; if(!lidar_only) pilot.tick(now,tx);
      for(const auto &f:tx) {
        if(raw_tx)raw_tx.write(reinterpret_cast<const char*>(f.data()),f.size());
        if(!serial.write(f.data(),f.size())) {pilot.stop("serial write failed or timed out");failed=true;break;}
      }
      if(pilot.phase()!=last_phase) {
        // Preview may have aged while GCS starts flight: never reuse it at takeover.
        if(planner&&pilot.phase()==Phase::PRE_OFFBOARD)planner->dropTrajectory();
        std::printf("[STATE] %.2fs %s (%s)\n",now-begin,phaseName(pilot.phase()),holdReasonName(pilot.holdReason()));last_phase=pilot.phase();}
      if(now-last_report>=1) {
        last_report=now; const auto st=async->stats(); const auto ls=lidar.stats(); const auto &ts=pilot.timeAlignment();
        std::printf("[STATUS] t=%.1f %s GPS=%u pose=%d sync=%d n=%d RTT=%.2fms reject=%llu lidar=%llu type=%u ptp=%d aligned=%llu map=%d loop_max=%.1fms CRC=%llu\n",now-begin,phaseName(pilot.phase()),pilot.gpsFixType(),pilot.poseFresh(now),pilot.clockReady(now),ts.samples(),ts.rtt()*1000,(unsigned long long)ts.rejected(),(unsigned long long)ls.packets,ls.time_type,ptp_ready,(unsigned long long)aligned,st.inflate_samples,max_loop*1000,(unsigned long long)pilot.crcErrors());
        if(diag) {diag<<std::setprecision(12)<<(now-begin)<<','<<phaseName(pilot.phase())<<','<<int(pilot.gpsFixType())<<','<<pilot.clockReady(now)<<','<<ts.samples()<<','<<ts.rtt()*1000<<','<<ts.residual()*1000<<','<<ls.packets<<','<<int(ls.time_type)<<','<<aligned<<','<<rejected<<','<<st.inflate_samples<<','<<pilot.poseFresh(now)<<','<<max_loop*1000<<','<<pilot.crcErrors()<<','<<ptp_ready<<','<<ls.time_offset_ns<<','<<ls.crc_missing<<','<<kml_ready<<','<<st.step_calls<<'\n';diag.flush();}

      }
      if(uplink.isOpen()&&now-last_uplink>=0.5) {
        last_uplink=now;GeoFix estimate;double age=0;const bool have=pilot.gpsEstimate(estimate,age);
        const auto p=pilot.latestPose().p;char line[768];
        std::snprintf(line,sizeof(line),"{\"t\":%.3f,\"phase\":\"%s\",\"hold\":\"%s\",\"fix\":%u,\"est_ok\":%s,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.3f,\"age\":%.3f,\"pos\":[%.3f,%.3f,%.3f],\"leg\":%d,\"clock_ok\":%s}\n",
          now-begin,phaseName(pilot.phase()),holdReasonName(pilot.holdReason()),pilot.gpsFixType(),have?"true":"false",estimate.lat_deg,estimate.lon_deg,estimate.alt_m,age,p.x(),p.y(),p.z(),pilot.currentLeg(),pilot.clockReady(now)?"true":"false");
        if(!uplink.sendLine(line)) ++uplink_failures;
      }
      if(failed||pilot.phase()==Phase::FAILED) {failed=true;break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto final_lidar=lidar.stats();
    lidar.stop(); serial.close();
    std::printf("[LIDAR] packets=%llu frames=%llu invalid=%llu queue_drops=%llu missing_crc=%llu (zero CRC means absent; packet acceptance follows configured policy)\n", (unsigned long long)final_lidar.packets,(unsigned long long)final_lidar.frames,(unsigned long long)final_lidar.invalid,(unsigned long long)final_lidar.queue_drops,(unsigned long long)final_lidar.crc_missing);
    const auto stats=async->stats();
    std::printf("[RESULT] phase=%s reason=%s error=%s maps=%d points=%ld plans=%d failures=%d aligned=%llu rejected=%llu max_loop=%.2fms\n",phaseName(pilot.phase()),holdReasonName(pilot.holdReason()),pilot.lastError(),stats.inflate_samples,stats.cloud_points,stats.replan,stats.plan_fail,(unsigned long long)aligned,(unsigned long long)rejected,max_loop*1000);
    std::printf("[UPLINK] failed datagrams=%llu\n",(unsigned long long)uplink_failures);
    for(const auto &e:pilot.crcErrorsByMsgid()) std::printf("[CRC] msg=%u failures=%llu\n",e.first,(unsigned long long)e.second);
    return (failed||(lidar_only&&final_lidar.frames==0))?1:0;
  } catch(const std::exception &e) { std::fprintf(stderr,"[FATAL] %s\n",e.what());return 2; }
}
