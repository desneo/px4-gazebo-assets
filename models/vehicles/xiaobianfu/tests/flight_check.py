#!/usr/bin/env python3
"""PX4 自动手抛起飞、满油门持续平飞与任务降落；Gazebo 无风真值验收。"""
import argparse,collections,hashlib,json,math,threading,time,subprocess
from pathlib import Path
from pymavlink import mavutil
ROOT=Path(__file__).resolve().parents[1]
class Truth:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.rows=collections.deque(maxlen=150);self.lock=threading.Lock();self.node=Node();self.last_wall=0
        self.node.subscribe(Pose_V,'/world/xiaobianfu_test/dynamic_pose/info',self.receive)
    def receive(self,msg):
        t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        for p in msg.pose:
            if p.name=='xiaobianfu':
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
            return {'t':b[0],'alt_m':b[3]-.04,'speed_m_s':math.sqrt(sum((b[i]-a[i])**2 for i in (1,2,3)))/dt,'climb_m_s':(b[3]-a[3])/dt,'age_s':time.monotonic()-self.last_wall}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'tests/flight_result.json');ap.add_argument('--config',type=Path,default=ROOT/'parameters.json');ap.add_argument('--model-dir',type=Path,default=ROOT);ap.add_argument('--wall-timeout',type=float,default=360);a=ap.parse_args();cfg=json.loads(a.config.read_text())
    report={'passed':False,'parameters':cfg,'config_sha256':hashlib.sha256(a.config.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256((a.model_dir/'model.sdf').read_bytes()).hexdigest(),'world_sha256':hashlib.sha256((a.model_dir/'tests/world.sdf').read_bytes()).hexdigest(),'airframe_sha256':hashlib.sha256((a.model_dir/'px4/4024_gz_xiaobianfu').read_bytes()).hexdigest(),'validator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'samples':[],'events':[],'criteria':{'target_m_s':cfg['target_airspeed_m_s'],'maximum_speed_upper_ratio':1.02,'altitude_tolerance_m':2,'continuous_sim_seconds':30,'max_sample_gap_s':.35,'touchdown_max_descent_m_s':1.8,'touchdown_max_bank_deg':15,'touchdown_pitch_range_deg':[-2,20]},'takeoff_passed':False,'landing_passed':False}
    m=mavutil.mavlink_connection('udpin:0.0.0.0:14540',source_system=255);m.wait_heartbeat(timeout=20);truth=Truth();start=time.monotonic();latest={};received={};last_hb=last_send=0;stage='ready';stage_t=None;integral=0;last_pos_t=-1;continuous=[];cruise_pass=False;launched=False;takeoff_wall=0;ground_stable=None;offboard_requested=False;landing_previous=None;touchdown=None
    def cmd(command,*params):m.mav.command_long_send(m.target_system,m.target_component,command,0,*(list(params)+[0]*7)[:7])
    def mode(main,sub=0):cmd(176,1,main,sub)
    def attitude(pitch,thrust):
        yaw=math.pi/2;c=math.cos(pitch/2);s=math.sin(pitch/2);cy=math.cos(yaw/2);sy=math.sin(yaw/2)
        m.mav.set_attitude_target_send(0,m.target_system,m.target_component,7,[c*cy,-s*sy,s*cy,c*sy],0,0,0,thrust)
    def landing_mission():
        gps=latest['GLOBAL_POSITION_INT'];pos=latest['LOCAL_POSITION_NED'];lat=gps.lat/1e7;lon=gps.lon/1e7;norm=max(1,math.hypot(pos.vx,pos.vy));north=pos.vx/norm;east=pos.vy/norm;ground_msl=gps.alt/1000-truth.sample()['alt_m']
        def point(distance,alt):
            correction=((pos.x+north*distance)**2+(pos.y+east*distance)**2-pos.x**2-pos.y**2)/(2*6371000)
            return (int((lat+north*distance/111111)*1e7),int((lon+east*distance/(111111*math.cos(math.radians(lat))))*1e7),ground_msl+correction+alt)
        # 落地区位于当前航迹前方；明确的进近航点与 LAND 构成自动降落任务。
        items=[(189,point(0,50)),(16,point(200,50)),(21,point(850,0))]
        m.mav.mission_count_send(m.target_system,m.target_component,len(items));deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            attitude(pitch,1)
            msg=m.recv_match(blocking=True,timeout=.05)
            m.mav.heartbeat_send(6,8,0,0,4)
            if not msg:continue
            if msg.get_type() in ('MISSION_REQUEST','MISSION_REQUEST_INT'):
                seq=msg.seq;command,(lat_i,lon_i,alt)=items[seq]
                m.mav.mission_item_int_send(m.target_system,m.target_component,seq,(mavutil.mavlink.MAV_FRAME_MISSION if command==189 else mavutil.mavlink.MAV_FRAME_GLOBAL_INT),command,0,1,0,0,0,float('nan'),lat_i,lon_i,alt)
            elif msg.get_type()=='MISSION_ACK':
                if msg.type!=0:raise RuntimeError('降落任务被拒绝: '+str(msg))
                mode(4,4);return
        raise RuntimeError('上传降落任务超时')
    try:
        for msgid in (32,74,30,33,245):cmd(511,msgid,100000)
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
                cmd(22,0,0,0,90,float('nan'),float('nan'),50);cmd(400,1);stage='takeoff';stage_t=pos.time_boot_ms/1000;takeoff_wall=now
            if stage=='takeoff' and not launched and now-takeoff_wall>1:
                if not hb or not hb.base_mode&128:raise RuntimeError('起飞解锁失败')
                subprocess.run(['sh',str(Path(__file__).with_name('hand_launch.sh'))],check=True);launched=True
            if not pos or not vfr:continue
            t=pos.time_boot_ms/1000;estimated_alt=-pos.z;tr_control=truth.sample();alt=tr_control['alt_m'] if tr_control and tr_control['age_s']<.35 else estimated_alt;climb=tr_control['climb_m_s'] if tr_control and tr_control['age_s']<.35 else -pos.vz
            if stage=='takeoff' and alt>45:
                report['takeoff_passed']=True;stage='max_power';stage_t=t
            if stage=='max_power' and now-last_send>.05:
                integral=max(-.12,min(.12,integral+(50-alt)*.00003));pitch=max(-.12,min(.30,.02+.006*(50-alt)-.05*climb+integral));attitude(pitch,1)
                if t-stage_t>1 and not offboard_requested:mode(6);offboard_requested=True
                last_send=now
            if t<=last_pos_t:continue
            last_pos_t=t;tr=truth.sample();mode_name=mavutil.interpret_px4_mode(hb.base_mode,hb.custom_mode) if hb else ''
            row={'t':t,'wall_s':now-start,'stage':stage,'alt_m':alt,'estimated_alt_m':estimated_alt,'estimator_alt_error_m':estimated_alt-alt,'climb_m_s':climb,'airspeed_m_s':vfr.airspeed,'groundspeed_m_s':vfr.groundspeed,'throttle_pct':vfr.throttle,'x_m':pos.x,'y_m':pos.y,'mode':mode_name,'truth':tr};report['samples'].append(row)
            if len(report['samples'])%50==0:print(row,flush=True)
            if stage=='takeoff' and t-stage_t>100:raise RuntimeError('手抛起飞未在100仿真秒内完成')
            if stage=='max_power':
                if alt<10:raise RuntimeError('满油门平飞高度失守')
                valid=now-received.get('VFR_HUD',0)<.5 and now-received.get('HEARTBEAT',0)<2 and tr and tr['age_s']<.35 and abs(tr['alt_m']-50)<=2 and abs(alt-50)<=2 and cfg['target_airspeed_m_s']<=tr['speed_m_s']<=cfg['target_airspeed_m_s']*1.02 and abs(vfr.airspeed-cfg['target_airspeed_m_s'])<=1.5 and abs(tr['climb_m_s'])<.3 and vfr.throttle>=99 and mode_name=='OFFBOARD'
                if not valid or (continuous and tr['t']-continuous[-1]['truth']['t']>.35):continuous=[]
                if valid and (not continuous or tr['t']>continuous[-1]['truth']['t']):continuous.append(row)
                if continuous and continuous[-1]['truth']['t']-continuous[0]['truth']['t']>=30:
                    cruise_pass=True;report['cruise']={'passed':True,'start_t':continuous[0]['truth']['t'],'end_t':continuous[-1]['truth']['t'],'min_true_speed_m_s':min(r['truth']['speed_m_s'] for r in continuous),'max_true_speed_m_s':max(r['truth']['speed_m_s'] for r in continuous),'max_true_alt_error_m':max(abs(r['truth']['alt_m']-50) for r in continuous)}
                    stage='landing';stage_t=t;landing_mission()
                elif t-stage_t>170:raise RuntimeError('满油门平飞未达到连续30仿真秒目标窗口')
            if stage=='landing':
                current_mode=mavutil.interpret_px4_mode(hb.base_mode,hb.custom_mode) if hb else ''
                if current_mode=='OFFBOARD':
                    attitude(pitch,1)
                    if now-last_send>.5:mode(4,4);last_send=now
                att=latest.get('ATTITUDE')
                if tr and tr['age_s']<.35 and att and now-received.get('ATTITUDE',0)<.2:
                    landing_row={'truth':dict(tr),'bank_deg':math.degrees(att.roll),'pitch_deg':math.degrees(att.pitch)}
                    if touchdown is None and landing_previous and tr['alt_m']<.15 and landing_previous['truth']['alt_m']>=.15:
                        touchdown=landing_previous
                        touchdown['passed']=bool(-.1<=-touchdown['truth']['climb_m_s']<=1.8 and abs(touchdown['bank_deg'])<=15 and -2<=touchdown['pitch_deg']<=20)
                        report['touchdown']=touchdown
                        if not touchdown['passed']:raise RuntimeError('接地下降速度或姿态不符合正常机腹着陆条件')
                    landing_previous=landing_row
                ext=latest.get('EXTENDED_SYS_STATE')
                ground_ok=bool(att and abs(math.degrees(att.roll))<=15 and -5<=math.degrees(att.pitch)<=25 and now-received.get('HEARTBEAT',0)<2 and now-received.get('ATTITUDE',0)<.5 and now-received.get('EXTENDED_SYS_STATE',0)<.5 and touchdown and touchdown['passed'] and tr and tr['age_s']<.35 and abs(tr['alt_m'])<.15 and tr['speed_m_s']<.3 and ext and ext.landed_state==1 and hb and not hb.base_mode&128)
                if not ground_ok:ground_stable=None
                elif ground_stable is None:ground_stable=tr['t']
                elif tr['t']-ground_stable>=3:
                    report['landing_passed']=True;report['landing_stop_sim_seconds']=tr['t']-ground_stable;report['passed']=report['takeoff_passed'] and cruise_pass;break
                if t-stage_t>140:raise RuntimeError('自动降落未在140仿真秒内落地停机')
        else:raise RuntimeError('飞行测试墙钟时间超时')
    except Exception as e:report['error']=str(e);print(e,flush=True)
    finally:
        if not report['landing_passed']:mode(4,3)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2));print(a.output,'passed=',report['passed'],flush=True)
if __name__=='__main__':main()
