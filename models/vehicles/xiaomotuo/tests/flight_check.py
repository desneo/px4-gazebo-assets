#!/usr/bin/env python3
"""PX4 自动滑跑起飞、满油门持续平飞与任务降落；Gazebo 无风真值验收。"""
import argparse,collections,hashlib,json,math,threading,time
from pathlib import Path
from pymavlink import mavutil
ROOT=Path(__file__).resolve().parents[1]
class Truth:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.rows=collections.deque(maxlen=150);self.lock=threading.Lock();self.node=Node();self.last_wall=0
        self.node.subscribe(Pose_V,'/world/xiaomotuo_test/dynamic_pose/info',self.receive)
    def receive(self,msg):
        t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        for p in msg.pose:
            if p.name=='xiaomotuo':
                with self.lock:
                    if self.rows and t<=self.rows[-1][0]:return
                    self.rows.append((t,p.position.x,p.position.y,p.position.z));self.last_wall=time.monotonic()
    def sample(self):
        with self.lock:
            if len(self.rows)<10:return None
            b=self.rows[-1];a=self.rows[0]
            for row in reversed(self.rows):
                if b[0]-row[0]>=.2:a=row;break
            dt=b[0]-a[0]
            if dt<=0:return None
            return {'t':b[0],'alt_m':b[3]-.55,'speed_m_s':math.sqrt(sum((b[i]-a[i])**2 for i in (1,2,3)))/dt,'climb_m_s':(b[3]-a[3])/dt,'age_s':time.monotonic()-self.last_wall}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'tests/flight_result.json');ap.add_argument('--config',type=Path,default=ROOT/'parameters.json');ap.add_argument('--model-dir',type=Path,default=ROOT);ap.add_argument('--wall-timeout',type=float,default=360);ap.add_argument('--resume-report',type=Path);ap.add_argument('--settings',type=Path,default=ROOT/'tests/settings.json');a=ap.parse_args();cfg=json.loads(a.config.read_text());settings=json.loads(a.settings.read_text())
    if not isinstance(settings['landing_timeout_sim_s'],(int,float)) or not math.isfinite(settings['landing_timeout_sim_s']) or settings['landing_timeout_sim_s']<=0:raise ValueError('landing_timeout_sim_s必须为有限正数')
    report={'validator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'test_settings':settings,'test_settings_sha256':hashlib.sha256(a.settings.read_bytes()).hexdigest(),'passed':False,'parameters':cfg,'config_sha256':hashlib.sha256(a.config.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256((a.model_dir/'model.sdf').read_bytes()).hexdigest(),'samples':[],'events':[],'criteria':{'target_m_s':cfg['target_airspeed_m_s'],'true_speed_interval_m_s':[cfg['target_airspeed_m_s'],cfg['target_airspeed_m_s']*1.02],'altitude_tolerance_m':2,'continuous_sim_seconds':30,'max_sample_gap_s':.35},'takeoff_passed':False,'landing_passed':False}
    if a.resume_report:
        report=json.loads(a.resume_report.read_text());report['passed']=False;report.pop('error',None);report['landing_retry']=True
        if report['config_sha256']!=hashlib.sha256(a.config.read_bytes()).hexdigest() or report['model_sha256']!=hashlib.sha256((a.model_dir/'model.sdf').read_bytes()).hexdigest():raise RuntimeError('续接降落配置与先前飞行证据不一致')
    m=mavutil.mavlink_connection('udpin:0.0.0.0:14540',source_system=255);m.wait_heartbeat(timeout=20);truth=Truth();start=time.monotonic();latest={};received={};stop_start=None;touchdown_checked=False;last_mode_retry=0;last_hb=last_send=0;stage='ready';stage_t=None;integral=0;last_pos_t=-1;continuous=[];cruise_pass=report.get('cruise',{}).get('passed',False)
    def cmd(command,*params):m.mav.command_long_send(m.target_system,m.target_component,command,0,*(list(params)+[0]*7)[:7])
    def mode(main,sub=0):cmd(176,1,main,sub)
    def attitude(pitch,thrust):
        yaw=math.pi/2;c=math.cos(pitch/2);s=math.sin(pitch/2);cy=math.cos(yaw/2);sy=math.sin(yaw/2)
        m.mav.set_attitude_target_send(0,m.target_system,m.target_component,7,[c*cy,-s*sy,s*cy,c*sy],0,0,0,thrust)
    def configure_landing():
        report['verified_runtime_parameters']={}
        for name,expected in cfg['px4'].items():
            deadline=time.monotonic()+5;last=0;verified=False
            while time.monotonic()<deadline:
                now=time.monotonic()
                if now-last>1:
                    m.mav.param_request_read_send(m.target_system,m.target_component,name.encode(),-1);last=now
                m.mav.heartbeat_send(6,8,0,0,4)
                attitude(latest.get('ATTITUDE').pitch if latest.get('ATTITUDE') else .02,1)
                msg=m.recv_match(blocking=True,timeout=.05)
                if msg and msg.get_type()=='PARAM_VALUE' and msg.param_id==name:
                    if not math.isclose(msg.param_value,expected,abs_tol=1e-5):raise RuntimeError(f'{name}实际值{msg.param_value}与配置{expected}不一致')
                    report['verified_runtime_parameters'][name]=msg.param_value;verified=True;break
            if not verified:raise RuntimeError('未确认'+name+'实际生效')
        report['startup_airframe_sha256']=hashlib.sha256((a.model_dir/'px4/4030_gz_xiaomotuo').read_bytes()).hexdigest()
    def landing_mission():
        gps=latest['GLOBAL_POSITION_INT'];pos=latest['LOCAL_POSITION_NED'];lat=gps.lat/1e7;lon=gps.lon/1e7;norm=max(1,math.hypot(pos.vx,pos.vy));north=pos.vx/norm;east=pos.vy/norm;ground_msl=gps.alt/1000-truth.sample()['alt_m']
        def point(distance,alt):
            # Gazebo 平面世界映射至 WGS84 时，离切平面原点越远地理高度越高。
            correction=((pos.x+north*distance)**2+(pos.y+east*distance)**2-pos.x**2-pos.y**2)/(2*6371000)
            return (int((lat+north*distance/111111)*1e7),int((lon+east*distance/(111111*math.cos(math.radians(lat))))*1e7),ground_msl+correction+alt)
        # 落地区位于当前航迹前方；明确的进近航点与 LAND 构成自动降落任务。
        items=[(189,point(0,100)),(16,point(400,100)),(21,point(1550,0))]
        report['landing_mission_items']=[{'command':command,'latitude_int':p[0],'longitude_int':p[1],'altitude_msl_m':p[2]} for command,p in items]
        m.mav.mission_count_send(m.target_system,m.target_component,len(items));deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            msg=m.recv_match(blocking=True,timeout=.1)
            m.mav.heartbeat_send(6,8,0,0,4)
            attitude(latest.get('ATTITUDE').pitch if latest.get('ATTITUDE') else .02,1)
            if not msg:continue
            if msg.get_type() in ('MISSION_REQUEST','MISSION_REQUEST_INT'):
                seq=msg.seq;command,(lat_i,lon_i,alt)=items[seq]
                m.mav.mission_item_int_send(m.target_system,m.target_component,seq,mavutil.mavlink.MAV_FRAME_MISSION if command==189 else mavutil.mavlink.MAV_FRAME_GLOBAL_INT,command,0,1,0,0,0,float('nan'),lat_i,lon_i,alt)
            elif msg.get_type()=='MISSION_ACK':
                if msg.type!=0:raise RuntimeError('降落任务被拒绝: '+str(msg))
                mode(4,4);return
        raise RuntimeError('上传降落任务超时')
    try:
        for msgid in (32,74,30,33,245,83):cmd(511,msgid,100000)
        while time.monotonic()-start<a.wall_timeout:
            now=time.monotonic()
            if now-last_hb>1:m.mav.heartbeat_send(6,8,0,0,4);last_hb=now
            msg=m.recv_match(blocking=True,timeout=.01)
            if msg:
                typ=msg.get_type();latest[typ]=msg;received[typ]=now
                if typ in ('COMMAND_ACK','STATUSTEXT'):
                    report['events'].append(str(msg));print(msg,flush=True)
            pos=latest.get('LOCAL_POSITION_NED');vfr=latest.get('VFR_HUD');hb=latest.get('HEARTBEAT')
            if stage=='ready' and now-start>8:
                if truth.sample() is None:raise RuntimeError('未收到 Gazebo 真值')
                if a.resume_report:
                    configure_landing()
                    if not cruise_pass or not report.get('takeoff_passed'):raise RuntimeError('续接降落缺少有效起飞/平飞证据')
                    stage='landing';stage_t=pos.time_boot_ms/1000;landing_mission()
                else:cmd(22,0,0,0,90,float('nan'),float('nan'),100);cmd(400,1);stage='takeoff';stage_t=pos.time_boot_ms/1000
            if not pos or not vfr:continue
            t=pos.time_boot_ms/1000;alt=-pos.z;climb=-pos.vz
            if stage=='takeoff' and alt>90:
                report['takeoff_passed']=True;stage='max_power';stage_t=t
            if stage=='max_power' and now-last_send>.05:
                guidance=truth.sample()
                if guidance is None or guidance['age_s']>.35:raise RuntimeError('满油门试验真值失联')
                control_alt=guidance['alt_m'];control_climb=guidance['climb_m_s']
                integral=max(-.12,min(.12,integral+(100-control_alt)*.00003));pitch=max(-.12,min(.30,.02+.006*(100-control_alt)-.05*control_climb+integral));attitude(pitch,1)
                if t-stage_t>1 and t-stage_t<2:mode(6)
                last_send=now
            if t<=last_pos_t:continue
            last_pos_t=t;tr=truth.sample();mode_name=mavutil.interpret_px4_mode(hb.base_mode,hb.custom_mode) if hb else ''
            att=latest.get('ATTITUDE');att_fresh=att and now-received.get('ATTITUDE',0)<.5
            target=latest.get('ATTITUDE_TARGET');target_fresh=target and now-received.get('ATTITUDE_TARGET',0)<.5
            target_pitch=math.degrees(math.asin(max(-1,min(1,2*(target.q[0]*target.q[2]-target.q[3]*target.q[1]))))) if target_fresh else None
            row={'target_pitch_deg':target_pitch,'roll_deg':math.degrees(att.roll) if att_fresh else None,'pitch_deg':math.degrees(att.pitch) if att_fresh else None,'t':t,'wall_s':now-start,'stage':stage,'alt_m':alt,'climb_m_s':climb,'airspeed_m_s':vfr.airspeed,'groundspeed_m_s':vfr.groundspeed,'throttle_pct':vfr.throttle,'x_m':pos.x,'y_m':pos.y,'mode':mode_name,'truth':tr,'estimator_alt_error_m':alt-tr['alt_m'] if tr else None};report['samples'].append(row)
            if len(report['samples'])%50==0:print(row,flush=True)
            if stage=='takeoff' and t-stage_t>100:raise RuntimeError('滑跑起飞未在100仿真秒内完成')
            if stage=='max_power':
                if alt<10:raise RuntimeError('满油门平飞高度失守')
                valid=tr and tr['age_s']<.35 and all(now-received.get(k,0)<1.5 for k in ('VFR_HUD','HEARTBEAT','LOCAL_POSITION_NED')) and abs(tr['alt_m']-100)<=2 and cfg['target_airspeed_m_s']<=tr['speed_m_s']<=cfg['target_airspeed_m_s']*1.02 and abs(vfr.airspeed-cfg['target_airspeed_m_s'])<=1.5 and abs(tr['climb_m_s'])<.3 and vfr.throttle>=99 and mode_name=='OFFBOARD'
                if not valid or (continuous and tr['t']-continuous[-1]['truth']['t']>.35):continuous=[]
                if valid and (not continuous or tr['t']>continuous[-1]['truth']['t']):continuous.append(row)
                if continuous and continuous[-1]['truth']['t']-continuous[0]['truth']['t']>=30:
                    cruise_pass=True;report['cruise']={'passed':True,'start_t':continuous[0]['truth']['t'],'end_t':continuous[-1]['truth']['t'],'min_true_speed_m_s':min(r['truth']['speed_m_s'] for r in continuous),'max_true_speed_m_s':max(r['truth']['speed_m_s'] for r in continuous),'max_true_alt_error_m':max(abs(r['truth']['alt_m']-100) for r in continuous),'max_estimator_alt_error_m':max(abs(r['estimator_alt_error_m']) for r in continuous)}
                    stage='landing';stage_t=t;configure_landing();landing_mission()
                elif t-stage_t>170:raise RuntimeError('满油门平飞未达到连续30仿真秒目标窗口')
            if stage=='landing':
                if mode_name not in ('MISSION','LAND') and t-stage_t<5 and now-last_send>.05:
                    attitude(latest.get('ATTITUDE').pitch if latest.get('ATTITUDE') else .02,1)
                    last_send=now
                if t-stage_t<5 and mode_name!='MISSION' and now-last_mode_retry>1:
                    mode(4,4);last_mode_retry=now
                if t-stage_t>10 and mode_name not in ('MISSION','LAND'):
                    raise RuntimeError('PX4未进入降落任务模式')
                ext=latest.get('EXTENDED_SYS_STATE')
                if not touchdown_checked and tr and tr['alt_m']<.1:
                    touchdown_checked=True;pre=[r for r in report['samples'] if r['truth'] and tr['t']-.8<=r['truth']['t']<=tr['t']]
                    if pre and all(r['roll_deg'] is not None and r['pitch_deg'] is not None for r in pre):
                        metrics={'first_contact_t':tr['t'],'contact_climb_m_s':tr['climb_m_s'],'min_climb_m_s':min(r['truth']['climb_m_s'] for r in pre),'max_abs_roll_deg':max(abs(r['roll_deg']) for r in pre),'min_pitch_deg':min(r['pitch_deg'] for r in pre),'max_pitch_deg':max(r['pitch_deg'] for r in pre)}
                        metrics['passed']=metrics['contact_climb_m_s']>=-2 and metrics['min_climb_m_s']>=-3 and metrics['max_abs_roll_deg']<15 and metrics['min_pitch_deg']>-8 and metrics['max_pitch_deg']<20;report['touchdown']=metrics
                    else:report['touchdown']={'passed':False,'error':'接地姿态遥测不完整'}
                stopped=tr and tr['age_s']<.35 and abs(tr['alt_m'])<.15 and tr['speed_m_s']<.3 and ext and ext.landed_state==1 and hb and not hb.base_mode&128 and all(now-received.get(k,0)<2 for k in ('EXTENDED_SYS_STATE','HEARTBEAT','LOCAL_POSITION_NED'))
                if stopped:
                    if stop_start is None:stop_start=tr['t']
                    if tr['t']-stop_start>=3:
                        report['landing_passed']=report.get('touchdown',{}).get('passed',False);report['stopped_seconds']=tr['t']-stop_start;report['passed']=report['takeoff_passed'] and cruise_pass and report['landing_passed'];break
                else:stop_start=None
                if t-stage_t>settings['landing_timeout_sim_s']:raise RuntimeError(f"自动降落未在{settings['landing_timeout_sim_s']}仿真秒内落地停机")
        else:raise RuntimeError('飞行测试墙钟时间超时')
    except Exception as e:report['error']=str(e);print(e,flush=True)
    finally:
        if not report['landing_passed']:
            final_truth=truth.sample()
            if final_truth is not None and final_truth['alt_m']>.5:mode(4,3)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2));print(a.output,'passed=',report['passed'],flush=True)
if __name__=='__main__':main()
