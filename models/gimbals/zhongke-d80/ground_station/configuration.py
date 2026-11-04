"""地面站参数和连接配置；解析不导入OpenCV、GI或Gazebo。"""
from __future__ import annotations
import argparse
import configparser
import hashlib
import ipaddress
import math
from pathlib import Path
from urllib.parse import urlsplit

from common.config import load_config

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_STATION_CONFIG=Path(__file__).with_name('config.ini')
DEFAULT_CONNECTION=dict(udp_host='127.0.0.1',udp_port='8870',local_bind='0.0.0.0',
    local_port='0',rtsp_url='rtsp://127.0.0.1:8556/zhongke_d80_1',parameters_file='../parameters.ini')


def _ip(value,label):
    try:return str(ipaddress.ip_address(str(value).strip()))
    except ValueError as exc:raise ValueError(f'{label}必须为合法IP地址') from exc


def _port(value,label,allow_zero=False):
    if isinstance(value,bool) or not str(value).strip().isdigit():
        raise ValueError(f'{label}必须为整数端口')
    value=int(value)
    low=0 if allow_zero else 1
    if not low<=value<=65535:raise ValueError(f'{label}必须在{low}～65535内')
    return value


def _rtsp(value):
    try:
        parsed=urlsplit(value)
        if parsed.scheme!='rtsp' or not parsed.hostname or any(c.isspace() for c in value):
            raise ValueError
        if parsed.port is not None and not 1<=parsed.port<=65535:raise ValueError
    except ValueError as exc:raise ValueError('RTSP地址必须为合法rtsp://主机[:端口]/路径') from exc
    return value


def parse_arguments(argv=None):
    parser=argparse.ArgumentParser(description='中科D80独立二进制地面站')
    parser.add_argument('mode',nargs='?',choices=('view',),help='兼容view config.ini入口')
    parser.add_argument('legacy_config',nargs='?',help='view模式的地面站配置路径')
    parser.add_argument('--station-config',help='地面站连接和显示INI；默认本程序config.ini')
    parser.add_argument('--session',help='显式读取仿真会话端点；连接CLI可覆盖')
    parser.add_argument('--config',help='独立模式的性能INI覆盖路径')
    parser.add_argument('--udp-host',help='覆盖远端UDP地址')
    parser.add_argument('--udp-port',help='覆盖远端UDP端口')
    parser.add_argument('--rtsp-url',help='覆盖RTSP视频地址')
    parser.add_argument('--window-title',help='覆盖窗口标题')
    parser.add_argument('--duration',type=float,default=0,help='运行秒数，0持续运行')
    parser.add_argument('--screenshot','--snapshot',dest='screenshot',help='截图绝对路径')
    parser.add_argument('--selftest',action='store_true',help='无窗口检查UDP和NVDEC')
    parser.add_argument('--decode-test',action='store_true',help='只检查RTSP，不建立控制租约')
    args=parser.parse_args(argv)
    if args.legacy_config and args.mode!='view':parser.error('配置位置参数须使用view config.ini')
    if args.legacy_config and args.station_config:
        parser.error('view配置路径与--station-config不能同时指定')
    args.runtime_connection=any(getattr(args,key) is not None for key in
        ('udp_host','udp_port','rtsp_url','window_title'))
    args.save_preferences=bool(args.station_config or args.legacy_config or not args.runtime_connection)
    args.station_config=str(Path(args.station_config or args.legacy_config or
                                 DEFAULT_STATION_CONFIG).expanduser().resolve())
    if args.config:args.config=str(Path(args.config).expanduser().resolve())
    if not math.isfinite(args.duration) or args.duration<0:parser.error('运行秒数须为非负有限数')
    return args


def station_preferences(path, overrides=None):
    path=Path(path).expanduser().resolve()
    parser=configparser.ConfigParser(interpolation=None,strict=True)
    try:
        with path.open(encoding='utf-8') as stream:parser.read_file(stream)
        allowed={'connection':set(DEFAULT_CONNECTION),
                 'video':{'display_scale','decoder','window_title','font_path'},
                 'forward':{'ip','port'}}
        if set(parser.sections())-set(allowed):raise ValueError('存在未知配置节，只允许connection/video/forward')
        for section in parser.sections():
            if set(parser[section])-allowed[section]:raise ValueError(f'{section}存在未知配置项')
        connection={key:parser.get('connection',key,fallback=default).strip()
                    for key,default in DEFAULT_CONNECTION.items()}
        overrides=overrides or {}
        connection.update({key:value for key,value in overrides.items() if key in connection})
        connection['udp_host']=_ip(connection['udp_host'],'远端UDP地址')
        connection['udp_port']=_port(connection['udp_port'],'远端UDP端口')
        connection['local_bind']=_ip(connection['local_bind'],'本地接收绑定地址')
        connection['local_port']=_port(connection['local_port'],'本地接收端口',allow_zero=True)
        connection['rtsp_url']=_rtsp(connection['rtsp_url'])
        if not connection['parameters_file']:raise ValueError('parameters_file不能为空')
        parameter=Path(connection['parameters_file']).expanduser()
        connection['parameters_file']=str((path.parent/parameter).resolve() if not parameter.is_absolute() else parameter.resolve())
        scale=parser.getfloat('video','display_scale',fallback=.5)
        if not math.isfinite(scale) or not .05<=scale<=2:raise ValueError('显示缩放须在0.05～2.0内')
        decoder=parser.get('video','decoder',fallback='nvdec')
        if decoder!='nvdec':raise ValueError('地面站仅支持nvdec硬件解码')
        title=overrides.get('window_title',parser.get('video','window_title',fallback='中科 D80 地面站')).strip()
        if not title:raise ValueError('窗口标题不能为空')
        return dict(**connection,station_config=str(path),display_scale=scale,decoder=decoder,
            window_title=title,font_path=parser.get('video','font_path',fallback='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'),
            forward_ip=_ip(parser.get('forward','ip',fallback='127.0.0.1'),'JSON接收地址'),
            forward_port=_port(parser.get('forward','port',fallback='9000'),'JSON接收端口'))
    except (OSError,ValueError,configparser.Error) as exc:
        raise ValueError(f'地面站配置读取失败：{path}：{exc}') from exc


def client_manifest_path(station_config):
    canonical=str(Path(station_config).expanduser().resolve())
    key=hashlib.sha256(canonical.encode('utf-8')).hexdigest()[:16]
    return ROOT/'runs'/('station_'+key)/'session.ini'


def resolve_configuration(args):
    session=None
    overrides={}
    if args.session:
        from scripts.session import load_session
        session=load_session(args.session)
        overrides.update({key:session[key] for key in ('udp_host','udp_port','rtsp_url')})
    overrides.update({key:getattr(args,key) for key in
        ('udp_host','udp_port','rtsp_url','window_title') if getattr(args,key,None) is not None})
    prefs=station_preferences(args.station_config,overrides)
    config_path=session['config_path'] if session else args.config or prefs['parameters_file']
    cfg=load_config(config_path)
    if session:
        if cfg['config_hash']!=session['config_hash']:raise ValueError('地面站性能配置与运行会话不匹配')
    if ipaddress.ip_address(prefs['udp_host']).version!=ipaddress.ip_address(prefs['local_bind']).version:
        raise ValueError('本地绑定与远端UDP地址的IP版本必须一致')
    if ipaddress.ip_address(prefs['udp_host']).is_unspecified:
        raise ValueError('远端UDP地址不能是0.0.0.0或::，请填写实际对端IP')
    prefs['parameters_file']=str(Path(config_path).expanduser().resolve())
    args.config=prefs['parameters_file']
    cfg['network'].update(udp_bind=prefs['udp_host'],udp_port=prefs['udp_port'])
    cfg['_rtsp_url']=prefs['rtsp_url']
    return cfg,prefs,session
