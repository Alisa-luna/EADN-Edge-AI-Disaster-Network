"""
地震预警服务器 v7.0 + CENC 实时目录接入
功能：
1. 接收节点直传的 eq_event 数据，多节点震源定位，存入 SQLite
2. 中国地震台网中心(CENC)实时目录监听，自动计算本地预估烈度
3. 烈度 > 4 时通过 MQTT、邮件、钉钉机器人推送告警
4. 烈度分布图生成，历史事件数据库
"""

import json
import math
import logging
import threading
import time
import sqlite3
import os
import ssl
from collections import defaultdict
from datetime import datetime
from typing import Optional
import numpy as np
import requests
import paho.mqtt.client as mqtt
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.header import Header

# ==================== 日志配置 ====================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('eq_server.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ==================== 全局配置 ====================
# MQTT
MQTT_BROKER = "**************************"
MQTT_PORT = 8883
MQTT_USERNAME = "User"
MQTT_PASSWORD = "1234567890"
MQTT_CA_CERT = "C:/Users/L1370/Desktop/main/emqxsl-ca.crt"  # 修改为实际路径
MQTT_TOPIC_DATA = "earthquake/data"
MQTT_TOPIC_ALERT = "earthquake/alert"
MQTT_TOPIC_HEARTBEAT = "earthquake/heartbeat"

# 邮箱
EMAIL_ENABLE = True
EMAIL_SENDER = "*********************"
EMAIL_PASSWORD = "****************"  # 授权码
EMAIL_RECEIVER = "*******************"
EMAIL_SMTP_SERVER = "smtp.qq.com"
EMAIL_SMTP_PORT = 587

# 事件关联与定位
EVENT_TIME_WINDOW = 30      # 秒
MIN_NODES_FOR_LOCATION = 2
GRID_SEARCH_STEP = 0.01
GRID_SEARCH_RADIUS = 0.5

# 高德地图
AMAP_KEY = "***********************"
AMAP_SECRET = "**********************"

# 服务器位置（用于CENC烈度计算）
STATION_LAT = 31.2304   # 改为实际坐标
STATION_LNG = 121.4737

# CENC 拉取间隔（秒）
CENC_FETCH_INTERVAL = 60

# 钉钉机器人 Token（从环境变量获取或直接填写）
DINGTALK_TOKEN = os.getenv("DINGTALK_TOKEN", "")

# ==================== 数据库模块 ====================
class EarthquakeDatabase:
    def __init__(self, db_path='earthquake.db'):
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.lock = threading.Lock()
        self._create_tables()

    def _create_tables(self):
        with self.lock:
            self.conn.executescript('''
                CREATE TABLE IF NOT EXISTS events (
                    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    start_epoch REAL,
                    epicenter_lat REAL,
                    epicenter_lng REAL,
                    magnitude REAL,
                    confidence REAL,
                    intensity INTEGER,
                    node_count INTEGER,
                    location_error_km REAL,
                    created_at TEXT
                );
                CREATE TABLE IF NOT EXISTS node_reports (
                    report_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    event_id INTEGER,
                    node_id INTEGER,
                    epoch REAL,
                    s_peak_gal REAL,
                    tau_c_sec REAL,
                    sp_time_sec REAL,
                    distance_km REAL,
                    warning_sec REAL,
                    ai_confidence REAL,
                    ae_anomaly REAL,
                    intensity INTEGER,
                    rssi INTEGER,
                    lat REAL,
                    lng REAL,
                    single_score REAL,
                    received_at REAL,
                    FOREIGN KEY(event_id) REFERENCES events(event_id)
                );
                CREATE TABLE IF NOT EXISTS hourly_baselines (
                    node_id INTEGER,
                    hour INTEGER,
                    mse_mean REAL,
                    mse_std REAL,
                    latent_mean REAL,
                    latent_std REAL,
                    updated_at REAL,
                    PRIMARY KEY (node_id, hour)
                );
            ''')
            self.conn.commit()

    def save_report(self, node_id, data, single_score=0):
        with self.lock:
            self.conn.execute('''INSERT INTO node_reports 
                (node_id, epoch, s_peak_gal, tau_c_sec, sp_time_sec, distance_km,
                 warning_sec, ai_confidence, ae_anomaly, intensity, rssi, lat, lng,
                 single_score, received_at)
                VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)''',
                (node_id, data['epoch'], data['s_peak'], data['tau_c'],
                 data['sp_time'], data['distance_km'], data['warning_sec'],
                 data['ai_conf'], data['ae_score'], data['intensity'],
                 data['rssi'], data['lat'], data['lng'],
                 single_score, time.time()))
            self.conn.commit()

    def create_event(self, start_epoch, epicenter_lat, epicenter_lng,
                     magnitude, confidence, intensity, node_count, error_km):
        with self.lock:
            cur = self.conn.execute('''INSERT INTO events 
                (start_epoch, epicenter_lat, epicenter_lng, magnitude, confidence,
                 intensity, node_count, location_error_km, created_at)
                VALUES (?,?,?,?,?,?,?,?,?)''',
                (start_epoch, epicenter_lat, epicenter_lng, magnitude, confidence,
                 intensity, node_count, error_km, datetime.now().isoformat()))
            self.conn.commit()
            return cur.lastrowid

    def link_reports(self, event_id, epoch_key):
        with self.lock:
            self.conn.execute('''UPDATE node_reports SET event_id=? 
                WHERE event_id IS NULL AND epoch >= ? AND epoch < ?''',
                (event_id, epoch_key * EVENT_TIME_WINDOW,
                 (epoch_key + 1) * EVENT_TIME_WINDOW))
            self.conn.commit()

    def get_event_count(self):
        return self.conn.execute('SELECT COUNT(*) FROM events').fetchone()[0]


# ==================== 地理工具 ====================
def haversine(lat1, lng1, lat2, lng2):
    R = 6371.0
    dlat = math.radians(lat2 - lat1)
    dlng = math.radians(lng2 - lng1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * \
        math.cos(math.radians(lat2)) * math.sin(dlng/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def locate_epicenter(reports, step=GRID_SEARCH_STEP, radius=GRID_SEARCH_RADIUS):
    if len(reports) < MIN_NODES_FOR_LOCATION:
        return None, None, None

    lats = [r['lat'] for r in reports]
    lngs = [r['lng'] for r in reports]
    center_lat = np.mean(lats)
    center_lng = np.mean(lngs)

    best_lat, best_lng = center_lat, center_lng
    best_error = float('inf')

    for dlat in np.arange(-radius, radius + step, step):
        for dlng in np.arange(-radius, radius + step, step):
            test_lat = center_lat + dlat
            test_lng = center_lng + dlng
            error = 0.0
            for r in reports:
                dist = haversine(test_lat, test_lng, r['lat'], r['lng'])
                error += (dist - r['distance_km'])**2
            if error < best_error:
                best_error = error
                best_lat = test_lat
                best_lng = test_lng

    error_km = math.sqrt(best_error / len(reports))
    return best_lat, best_lng, error_km


# ==================== 单节点评分 ====================
def compute_single_node_score(report):
    score = 0.0
    count = 0.0
    ai = report.get('ai_conf', 0)
    score += ai * 0.35
    count += 0.35

    ae = report.get('ae_score', 0)
    ae_norm = min(ae / 5.0, 1.0)
    score += ae_norm * 0.25
    count += 0.25

    s_peak = report.get('s_peak', 0)
    tau_c = report.get('tau_c', 0)
    if s_peak > 1 and tau_c > 0.01:
        expected_log = 0.5 * math.log10(s_peak) - 1.5
        actual_log = math.log10(tau_c)
        tau_error = abs(actual_log - expected_log)
        tau_score = math.exp(-tau_error * 2)
        score += tau_score * 0.20
        count += 0.20

    sp_time = report.get('sp_time', 0)
    dist = report.get('distance_km', 0)
    if sp_time > 0.1:
        expected_dist = 8.0 * sp_time
        sp_error = abs(dist - expected_dist)
        sp_score = math.exp(-sp_error / 20)
        score += sp_score * 0.20
        count += 0.20

    return score / count if count > 0 else 0.5


# ==================== 烈度分布图 ====================
def generate_shakemap_html(node_data, epicenter_lat, epicenter_lng, event_info=None):
    nodes_json = json.dumps([
        {
            'lat': n['lat'], 'lng': n['lng'],
            'int': n['intensity'],
            'pga': round(n.get('s_peak', 0), 1),
            'node_id': n.get('node_id', 0)
        } for n in node_data
    ], ensure_ascii=False)

    ev_time = event_info.get('time', '--') if event_info else '--'
    ev_mag = event_info.get('magnitude', '--') if event_info else '--'
    ev_conf = event_info.get('confidence', '--') if event_info else '--'

    html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>地震烈度分布</title>
<style>
    body,html{{margin:0;height:100%;}}
    #map{{width:100%;height:100%;}}
    .info-card{{
        position:absolute;top:10px;right:10px;background:rgba(0,0,0,0.8);color:#fff;
        padding:15px;border-radius:10px;font-family:Arial;font-size:14px;z-index:999;
    }}
</style></head><body>
<div id="map"></div>
<div class="info-card">
    <h3 style="color:#FF6B6B;margin:0">⚠️ 地震预警</h3>
    <p>🕐 {ev_time}</p>
    <p>📊 震级: M{ev_mag}</p>
    <p>📈 置信度: {ev_conf}%</p>
    <p>🔗 节点: {len(node_data)}</p>
</div>
<script>
window._AMapSecurityConfig={{securityJsCode:'{AMAP_SECRET}'}};
</script>
<script src="https://webapi.amap.com/maps?v=2.0&key={AMAP_KEY}&plugin=AMap.HeatMap"></script>
<script>
(function(){{
    var epicenter=[{epicenter_lng},{epicenter_lat}];
    var nodes={nodes_json};
    var map=new AMap.Map('map',{{zoom:12,center:epicenter,mapStyle:'amap://styles/darkblue'}});
    new AMap.Marker({{
        position:epicenter,
        content:'<div style="background:#e94560;color:#fff;padding:6px 12px;border-radius:20px;font-weight:bold;box-shadow:0 2px 8px rgba(0,0,0,0.3)">推定震中</div>',
        offset:new AMap.Pixel(-30,-40)
    }}).setMap(map);
    nodes.forEach(function(n){{
        var color=n.int>=7?'#e94560':n.int>=5?'#ff8c00':'#f5c518';
        new AMap.Marker({{
            position:[n.lng,n.lat],
            content:'<div style="background:'+color+';color:#fff;padding:4px 10px;border-radius:15px;font-size:13px;font-weight:bold">'+n.int+'度</div>',
            offset:new AMap.Pixel(-20,-20)
        }}).setMap(map);
    }});
    var heatmapData=nodes.map(function(n){{return{{lng:n.lng,lat:n.lat,count:n.int*10}};}});
    new AMap.HeatMap(map,{{
        radius:40,opacity:[0,0.8],
        gradient:{{0.2:'#4ecca3',0.4:'#f5c518',0.6:'#ff8c00',0.8:'#ff4500',1.0:'#e94560'}},
        dataSet:{{data:heatmapData,max:100}}
    }});
    map.setFitView();
}})();
</script></body></html>"""

    with open("shakemap.html", "w", encoding="utf-8") as f:
        f.write(html)
    logger.info("✅ 烈度分布图已生成: shakemap.html")
    return html


# ==================== 邮件发送 ====================
def send_email_alert(subject, html_content):
    if not EMAIL_ENABLE:
        return

    def _send():
        try:
            msg = MIMEMultipart('alternative')
            msg['From'] = EMAIL_SENDER
            msg['To'] = EMAIL_RECEIVER
            msg['Subject'] = Header(subject, 'utf-8')
            msg.attach(MIMEText(html_content, 'html', 'utf-8'))

            server = smtplib.SMTP(EMAIL_SMTP_SERVER, EMAIL_SMTP_PORT)
            server.starttls()
            server.login(EMAIL_SENDER, EMAIL_PASSWORD)
            server.send_message(msg)
            server.quit()
            logger.info(f"📧 邮件已发送: {subject}")
        except Exception as e:
            logger.error(f"❌ 邮件发送失败: {e}")

    threading.Thread(target=_send, daemon=True).start()


# ==================== 事件管理器（节点直传数据） ====================
class EventManager:
    def __init__(self, db, mqtt_client):
        self.db = db
        self.mqtt = mqtt_client
        self.pending = defaultdict(list)
        self.lock = threading.Lock()

    def add_report(self, node_id, report):
        epoch = report['epoch']
        key = int(epoch) // EVENT_TIME_WINDOW
        score = compute_single_node_score(report)
        self.db.save_report(node_id, report, score)

        with self.lock:
            self.pending[key].append({
                'node_id': node_id,
                'lat': report.get('lat', 0),
                'lng': report.get('lng', 0),
                'distance_km': report.get('distance_km', 0),
                's_peak': report.get('s_peak', 0),
                'intensity': report.get('intensity', 1),
                'score': score,
                'epoch': epoch
            })
            if len(self.pending[key]) >= MIN_NODES_FOR_LOCATION:
                self._trigger_event(key)


    def _single_node_epicenter(self, node_lat, node_lng, azimuth, distance_km):
        """
        单节点震中推算
        azimuth: P波初动方向（度，正北=0，顺时针）
        distance_km: 震中距（km）
        返回: (epicenter_lat, epicenter_lng)
        """
        # 震中在 P 波传播方向的反方向
        back_azimuth = azimuth + 180.0
        if back_azimuth >= 360.0:
            back_azimuth -= 360.0

        az_rad = math.radians(back_azimuth)

        # 纬度方向分量
        d_lat = distance_km * math.cos(az_rad) / 111.32

        # 经度方向分量（需根据纬度修正）
        d_lng = distance_km * math.sin(az_rad) / (111.32 * math.cos(math.radians(node_lat)))

        epi_lat = node_lat + d_lat
        epi_lng = node_lng + d_lng

        logger.info(f"📍 单节点推算震中: "
                    f"节点({node_lat:.4f},{node_lng:.4f}) "
                    f"方位角{azimuth:.1f}° 距离{distance_km:.1f}km "
                    f"→ 震中({epi_lat:.4f},{epi_lng:.4f})")

        return epi_lat, epi_lng

    def _trigger_event(self, key):
        reports = self.pending[key]

        epi_lat = None
        epi_lng = None
        error_km = None

        # ===== 判断用哪种定位方式 =====
        if len(reports) >= MIN_NODES_FOR_LOCATION:
            # 多节点：网格搜索定位
            epi_lat, epi_lng, error_km = locate_epicenter(reports)
            logger.info(f"📍 多节点定位: {len(reports)}个节点")

        elif len(reports) == 1:
            r = reports[0]
            # 单节点：用方位角推算
            if (r.get('azimuth', -1) >= 0 and
                    r.get('node_lat', 0) != 0 and
                    r.get('node_lng', 0) != 0):
                epi_lat, epi_lng = self._single_node_epicenter(
                    r['node_lat'], r['node_lng'],
                    r['azimuth'], r['distance_km']
                )
                error_km = r['distance_km'] * 0.3  # 单节点误差约 30%
                logger.info(f"📍 单节点定位: 方位角{r['azimuth']:.1f}°")
            else:
                # 没有方位角，以节点自身作为震中
                epi_lat = r.get('lat', 0)
                epi_lng = r.get('lng', 0)
                error_km = r['distance_km'] * 0.5
                logger.info(f"📍 无方位角，以节点位置作为震中参考")

        if epi_lat is None or epi_lng is None:
            return

        # ===== 综合置信度 =====
        scores = [r['score'] for r in reports]
        network_conf = np.mean(scores) * min(1.0, len(reports) / 3.0)

        # 单节点时降低置信度
        if len(reports) == 1:
            network_conf *= 0.7

        # ===== 震级融合 =====
        corrected = []
        for r in reports:
            d = max(r['distance_km'], 1.0)
            corrected.append(r['s_peak'] * (d / 10.0) ** 1.5)
        magnitude = np.mean(corrected) if corrected else 0
        intensity = max(r['intensity'] for r in reports)
        start_epoch = min(r['epoch'] for r in reports)

        # ===== 存入数据库 =====
        event_id = self.db.create_event(
            start_epoch, epi_lat, epi_lng, magnitude,
            network_conf, intensity, len(reports), error_km or 0
        )
        self.db.link_reports(event_id, key)

        # ===== 生成烈度图 =====
        node_data = [
            {
                'lat': r['lat'], 'lng': r['lng'],
                'intensity': r['intensity'],
                's_peak': r['s_peak'],
                'node_id': r['node_id']
            } for r in reports
        ]
        event_info = {
            'time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_epoch)),
            'magnitude': round(magnitude, 1),
            'confidence': round(network_conf * 100, 1),
            'nodes': len(reports),
            'method': '多节点网格搜索' if len(reports) >= 2 else '单节点方位角推算'
        }
        generate_shakemap_html(node_data, epi_lat, epi_lng, event_info)

        # ===== 构建邮件 =====
        node_rows = ""
        for r in reports:
            node_rows += f"<tr><td>Node {r['node_id']}</td>" \
                         f"<td>({r['lat']:.4f},{r['lng']:.4f})</td>" \
                         f"<td>{r['intensity']}度</td>" \
                         f"<td>{r['s_peak']:.1f} gal</td>" \
                         f"<td>{r['score']:.2f}</td></tr>"

        email_html = f"""
        <html><body>
        <h2>🔴 地震预警确认</h2>
        <hr>
        <table>
        <tr><td>📅 时间:</td><td>{event_info['time']}</td></tr>
        <tr><td>📍 推定震中:</td><td>{epi_lat:.4f}, {epi_lng:.4f}</td></tr>
        <tr><td>📊 震级:</td><td>M{event_info['magnitude']}</td></tr>
        <tr><td>📈 烈度:</td><td>{intensity}度</td></tr>
        <tr><td>🎯 置信度:</td><td>{event_info['confidence']}%</td></tr>
        <tr><td>🔗 触发节点:</td><td>{len(reports)}</td></tr>
        <tr><td>🔍 定位方式:</td><td>{event_info['method']}</td></tr>
        <tr><td>📍 定位误差:</td><td>{error_km:.1f} km</td></tr>
        </table>
        <hr>
        <h3>节点详情</h3>
        <table border="1" cellpadding="5" cellspacing="0" style="border-collapse:collapse">
        <tr><th>节点</th><th>坐标</th><th>烈度</th><th>PGA</th><th>评分</th></tr>
        {node_rows}
        </table>
        <hr>
        <p><small>自动发送 - 地震预警系统</small></p>
        </body></html>
        """

        send_email_alert(f"🔴 地震预警 M{event_info['magnitude']} {intensity}度", email_html)

        # ===== MQTT 警报 =====
        alert_payload = {
            "type": "earthquake_alert",
            "event_id": event_id,
            "epicenter_lat": epi_lat,
            "epicenter_lng": epi_lng,
            "magnitude": round(magnitude, 1),
            "intensity": intensity,
            "confidence": round(network_conf * 100, 1),
            "node_count": len(reports),
            "location_error_km": round(error_km or 0, 1),
            "method": event_info['method'],
            "timestamp": int(time.time())
        }
        self.mqtt.publish(MQTT_TOPIC_ALERT, json.dumps(alert_payload))
        logger.info(f"🚨 警报已发布: M{magnitude:.1f} {intensity}度 ({event_info['method']})")

        del self.pending[key]


# ==================== MQTT 处理 ====================
class MQTTHandler:
    def __init__(self, event_manager):
        self.event_mgr = event_manager
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        if MQTT_CA_CERT and os.path.exists(MQTT_CA_CERT):
            self.client.tls_set(ca_certs=MQTT_CA_CERT)
        else:
            self.client.tls_set(cert_reqs=ssl.CERT_NONE)
        self.connected = False

    def connect(self):
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
                self.client.loop_start()
                self.connected = True
                logger.info("📡 MQTT 已连接")
                return True
            except Exception as e:
                logger.error(f"❌ MQTT 连接失败 ({attempt+1}/{max_retries}): {e}")
                time.sleep(5)
        return False

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            client.subscribe(MQTT_TOPIC_DATA)
            client.subscribe(MQTT_TOPIC_ALERT)
            client.subscribe(MQTT_TOPIC_HEARTBEAT)
            logger.info("📥 已订阅所有 topic")
        else:
            logger.error(f"❌ MQTT 连接失败，错误码: {reason_code}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            msg_type = payload.get('type', '')
            if msg_type == 'eq_event':
                self._handle_eq_event(payload)
            elif msg_type == 'heartbeat':
                logger.debug(f"💓 心跳: {payload.get('nodes', 0)} 节点")
            elif msg_type == 'fused_event':
                logger.info(f"📊 网关融合事件: {payload}")
        except json.JSONDecodeError:
            logger.warning(f"⚠️ 非JSON消息: {msg.payload[:100]}")
        except Exception as e:
            logger.error(f"消息处理错误: {e}")

    def _handle_eq_event(self, data):
        node_id = data.get('node', 0)
        report = {
            'epoch': data.get('epoch', 0),
            's_peak': data.get('s_peak', 0),
            'tau_c': data.get('tau_c', 0),
            'sp_time': data.get('sp_time', 0),
            'distance_km': data.get('distance', 0),
            'warning_sec': data.get('warning', 0),
            'ai_conf': data.get('ai_confidence', 0),
            'ae_score': data.get('ae_anomaly', 0),
            'intensity': data.get('intensity', 1),
            'rssi': data.get('rssi', 0),
            'lat': data.get('gw_lat', 0),
            'lng': data.get('gw_lng', 0),
            # ===== 新增字段 =====
            'azimuth': data.get('azimuth', -1),  # P波方位角
            'node_lat': data.get('node_lat', 0),  # 节点自身纬度
            'node_lng': data.get('node_lng', 0),  # 节点自身经度
        }

        logger.info(f"📊 Node {node_id}: s_peak={report['s_peak']:.1f} "
                    f"τc={report['tau_c']:.3f} dist={report['distance_km']:.1f}km "
                    f"ai={report['ai_conf']:.2f} ae={report['ae_score']:.2f} "
                    f"az={report['azimuth']:.1f}°")

        self.event_mgr.add_report(node_id, report)

    def publish(self, topic, payload):
        if self.connected:
            self.client.publish(topic, payload)

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()


# ==================== CENC 实时目录接入 ====================
import websocket
import json
import time
import threading
from datetime import datetime


# ==================== 新 CENC 实时接入（WebSocket + REST） ====================
class CENCFetcherV2:
    """适配国家地震科学数据中心新 API v2"""

    def __init__(self):
        self.base_url = "https://api-cencint-public.nowquake.cn"
        self.last_id = None  # 记录最新事件ID
        self.last_update = None  # 记录最后更新时间
        self.known_events = set()  # 已处理事件集合
        self.event_callbacks = []

    def get_last_id(self):
        """获取最新事件ID"""
        try:
            url = f"{self.base_url}/lastid"
            resp = requests.get(url, timeout=5)
            if resp.status_code == 200:
                data = resp.json()
                return data.get('eq_id')
            return None
        except Exception as e:
            logger.error(f"获取最新ID失败: {e}")
            return None

    def get_last_update(self):
        """获取最后更新时间"""
        try:
            url = f"{self.base_url}/lastupdate"
            resp = requests.get(url, timeout=5)
            if resp.status_code == 200:
                data = resp.json()
                return data.get('update_time')
            return None
        except Exception as e:
            logger.error(f"获取更新时间失败: {e}")
            return None

    def fetch_recent_events(self, minutes=5):
        """获取最近的地震事件"""
        try:
            # 先获取最新事件ID
            current_last_id = self.get_last_id()

            # 如果ID没变化，说明没有新事件
            if current_last_id and current_last_id == self.last_id:
                logger.debug(f"📡 无新事件 (最新ID: {current_last_id})")
                return []

            # 有新事件，拉取列表
            events = self._fetch_event_list(minutes)

            # 更新最新ID
            if current_last_id:
                self.last_id = current_last_id

            return events

        except Exception as e:
            logger.error(f"❌ CENC API 错误: {e}")
            return self._fallback_usgs()

    def _fetch_event_list(self, minutes=None):
        """拉取事件列表，minutes=None 时不按时间过滤"""
        try:
            url = f"{self.base_url}/list"
            params = {'pageNo': 0, 'pageSize': 50}
            headers = {'User-Agent': 'EarthquakeWarningSystem/2.0', 'Accept': 'application/json'}

            resp = requests.get(url, params=params, headers=headers, timeout=10)
            if resp.status_code != 200:
                return []

            data = resp.json()
            if not isinstance(data, list):
                return []

            # 时间阈值（仅在 minutes 不为 None 时生效）
            time_threshold = None
            if minutes is not None:
                from datetime import datetime, timedelta
                time_threshold = datetime.now() - timedelta(minutes=minutes)

            events = []
            for item in data:
                if not isinstance(item, dict):
                    continue

                eq_id = item.get('eq_id', '')
                if not eq_id or eq_id in self.known_events:
                    continue

                # 时间过滤（仅在 minutes 不为 None 时）
                if time_threshold is not None:
                    happen_time_str = item.get('happen_time', '')
                    if happen_time_str:
                        try:
                            happen_time = datetime.strptime(happen_time_str, '%Y-%m-%d %H:%M:%S')
                            if happen_time < time_threshold:
                                continue
                        except:
                            pass

                self.known_events.add(eq_id)

                event = {
                    'id': eq_id,
                    'time': item.get('happen_time', ''),
                    'lat': float(item.get('latitude', 0)),
                    'lng': float(item.get('longitude', 0)),
                    'magnitude': float(item.get('magnitude', 0)),
                    'depth': float(item.get('depth', 10.0)),
                    'location': item.get('hypocenter', '未知'),
                    'max_intensity': float(item.get('maxintensity', 0)),
                    'instrumental_intensity': float(item.get('maxintensity', 0)),
                    'source': 'CENC',
                }
                events.append(event)

            return events

        except Exception as e:
            logger.error(f"获取事件列表失败: {e}")
            return []

    def fetch_event_detail(self, eq_id):
        """获取单个事件的详细信息（包含台站数据）"""
        try:
            url = f"{self.base_url}/event/{eq_id}"
            resp = requests.get(url, timeout=10)
            if resp.status_code == 200:
                detail = resp.json()
                return detail
            return None
        except Exception as e:
            logger.error(f"获取事件详情失败 {eq_id}: {e}")
            return None

    def register_callback(self, callback):
        """注册事件回调"""
        self.event_callbacks.append(callback)

    def _fallback_usgs(self):
        """备用数据源"""
        try:
            url = "https://earthquake.usgs.gov/earthquakes/feed/v1.0/summary/2.5_hour.geojson"
            resp = requests.get(url, timeout=15)

            if resp.status_code != 200:
                return []

            data = resp.json()
            events = []

            for feature in data['features'][:20]:
                props = feature['properties']
                eq_id = props.get('ids', props.get('id', ''))
                if not eq_id or eq_id in self.known_events:
                    continue
                self.known_events.add(eq_id)

                geom = feature['geometry']
                from datetime import datetime
                event_time = datetime.fromtimestamp(props['time'] / 1000)

                events.append({
                    'id': eq_id,
                    'time': event_time.strftime('%Y-%m-%d %H:%M:%S'),
                    'lat': geom['coordinates'][1],
                    'lng': geom['coordinates'][0],
                    'magnitude': props['mag'],
                    'depth': geom['coordinates'][2],
                    'location': props['place'],
                    'max_intensity': 0,
                    'instrumental_intensity': 0,
                    'source': 'USGS'
                })

            logger.info(f"📡 USGS 备用: {len(events)} 个事件")
            return events

        except Exception as e:
            logger.error(f"❌ USGS 获取失败: {e}")
            return []

# ==================== 烈度计算 ====================
class IntensityCalculator:
    def __init__(self, lat=STATION_LAT, lng=STATION_LNG):
        self.station_lat = lat
        self.station_lng = lng

    def calculate(self, mag, depth, eq_lat, eq_lng):
        distance = haversine(eq_lat, eq_lng, self.station_lat, self.station_lng)
        R = math.sqrt(distance**2 + depth**2)
        # 中国东部衰减模型
        I = 3.5 + 1.2*mag - 3.0*math.log10(R+1) - 0.002*R
        if depth > 70:
            I -= 1.5
        elif depth > 300:
            I -= 2.5
        intensity = max(1, min(12, int(round(I))))
        return intensity, distance


# ==================== 告警分发 ====================
class AlertDispatcher:
    def __init__(self, mqtt_handler, dingtalk_token=""):
        self.mqtt = mqtt_handler
        self.dingtalk_token = dingtalk_token
        self.alerted_events = set()

    def dispatch(self, event, intensity, distance):
        if event['id'] in self.alerted_events:
            return
        if intensity < 4:
            return
        self.alerted_events.add(event['id'])
        logger.warning(f"🚨 外部地震告警: M{event['magnitude']} 预估烈度{intensity}度 距离{distance:.0f}km")

        # MQTT
        payload = {
            "type": "external_alert",
            "source": "CENC",
            "event_id": event['id'],
            "time": event['time'],
            "location": event.get('location', '未知'),
            "magnitude": event['magnitude'],
            "depth_km": event['depth'],
            "distance_km": round(distance, 1),
            "estimated_intensity": intensity,
            "epicenter_lat": event['lat'],
            "epicenter_lng": event['lng'],
            "timestamp": int(time.time())
        }
        self.mqtt.publish("earthquake/alert", json.dumps(payload))

        # 邮件
        subject = f"⚠️ 外部地震预警: {event.get('location','未知')} M{event['magnitude']} 预估烈度{intensity}度"
        html = f"""
        <html><body>
        <h2>🌍 中国地震台网预警</h2>
        <hr>
        <table>
        <tr><td>📅 发震时间:</td><td>{event['time']}</td></tr>
        <tr><td>📍 震中位置:</td><td>{event.get('location','未知')}</td></tr>
        <tr><td>🎯 坐标:</td><td>{event['lat']:.2f}°N, {event['lng']:.2f}°E</td></tr>
        <tr><td>📊 震级:</td><td><b>M{event['magnitude']}</b></td></tr>
        <tr><td>📏 深度:</td><td>{event['depth']} km</td></tr>
        <tr><td>📏 震中距:</td><td>{distance:.1f} km</td></tr>
        <tr><td>📈 预估本地烈度:</td><td><b style="color:red;font-size:1.2em">{intensity}度</b></td></tr>
        </table>
        <hr>
        <p><small>自动发送 - 地震预警系统</small></p>
        </body></html>
        """
        send_email_alert(subject, html)

        # 钉钉
        if self.dingtalk_token:
            webhook = f"https://oapi.dingtalk.com/robot/send?access_token={self.dingtalk_token}"
            markdown_text = f"## ⚠️ 地震预警\n\n" \
                           f"- **发震时刻**: {event['time']}\n" \
                           f"- **震中位置**: {event.get('location','未知')}\n" \
                           f"- **震级**: M{event['magnitude']}\n" \
                           f"- **深度**: {event['depth']}km\n" \
                           f"- **震中距**: {distance:.0f}km\n" \
                           f"- **预估本地烈度**: {intensity}度\n"
            requests.post(webhook, json={"msgtype":"markdown","markdown":{"title":"地震预警","text":markdown_text}}, timeout=5)


# ==================== 更新烈度计算器（使用 CENC 仪器烈度） ====================
class IntensityCalculatorV2(IntensityCalculator):
    """
    增强版烈度计算器
    优先使用 CENC 提供的仪器烈度，否则使用经验公式
    """

    def calculate(self, mag, depth, eq_lat, eq_lng,
                  epicenter_intensity=0, instrumental_intensity=0):
        """
        计算本地预估烈度
        优先使用 CENC 提供的仪器烈度进行衰减计算
        """
        distance = haversine(eq_lat, eq_lng, self.station_lat, self.station_lng)

        # 如果 CENC 提供了仪器烈度，用它作为震中烈度进行衰减
        if instrumental_intensity > 0:
            # 仪器烈度衰减模型
            base_intensity = instrumental_intensity
        elif epicenter_intensity > 0:
            base_intensity = epicenter_intensity
        else:
            # 没有仪器烈度时使用震级经验公式
            R = math.sqrt(distance ** 2 + depth ** 2)
            base_intensity = 3.5 + 1.2 * mag - 3.0 * math.log10(R + 1) - 0.002 * R

        # 距离衰减：每增加 100km 烈度降低约 1-2 度
        attenuation = math.log10(max(distance, 1)) * 2.5
        estimated_intensity = base_intensity - attenuation

        # 深度修正
        if depth > 70:
            estimated_intensity -= 1.5
        elif depth > 300:
            estimated_intensity -= 2.5

        local_intensity = max(1, min(12, int(round(estimated_intensity))))

        return local_intensity, distance, base_intensity


# ==================== 更新告警分发器 ====================
class AlertDispatcherV2(AlertDispatcher):
    def dispatch(self, event, intensity, distance, epicenter_intensity=0):
        """发送告警，包含 CENC 提供的仪器烈度信息"""
        if event['id'] in self.alerted_events:
            return
        if intensity < 4:
            return

        self.alerted_events.add(event['id'])

        # 根据国标设置颜色
        color_map = {
            5: '#FFFFFF', 6: '#FFD2DA', 7: '#FFBEBE',
            8: '#FF7F7F', 9: '#C82828', 10: '#A80000', 11: '#8C0000'
        }
        color = color_map.get(intensity, '#FFFFFF')

        logger.warning(f"🚨 地震告警: {event.get('location', '未知')} M{event['magnitude']} "
                       f"仪器烈度{epicenter_intensity} 预估本地烈度{intensity}度")

        # MQTT 告警
        payload = {
            "type": "external_alert",
            "source": event.get('source', 'CENC'),
            "event_id": event['id'],
            "time": event['time'],
            "location": event.get('location', '未知'),
            "magnitude": event['magnitude'],
            "depth_km": event['depth'],
            "epicenter_intensity": epicenter_intensity,
            "instrumental_intensity": event.get('instrumental_intensity', 0),
            "estimated_local_intensity": intensity,
            "distance_km": round(distance, 1),
            "epicenter_lat": event['lat'],
            "epicenter_lng": event['lng'],
            "alert_color": color,
            "timestamp": int(time.time())
        }
        self.mqtt.publish("earthquake/alert", json.dumps(payload, ensure_ascii=False))

        # 邮件告警
        self._send_email_v2(event, intensity, distance, epicenter_intensity, color)

        # 钉钉告警
        if self.dingtalk_token:
            self._send_dingtalk_v2(event, intensity, distance, epicenter_intensity)

    def _send_email_v2(self, event, intensity, distance, epicenter_intensity, color):
        roman_map = {5: 'V', 6: 'VI', 7: 'VII', 8: 'VIII', 9: 'IX', 10: 'X', 11: 'XI', 12: 'XII'}
        roman = roman_map.get(epicenter_intensity, str(epicenter_intensity))

        subject = f"⚠️ 地震预警: {event.get('location', '未知')} M{event['magnitude']} 仪器烈度{roman}度"
        html = f"""
        <html><body>
        <h2>🌍 地震预警信息</h2>
        <p style="color:#888;font-size:12px">数据来源: 国家地震科学数据中心</p>
        <hr>
        <table style="border-collapse:collapse;width:100%">
        <tr><td>📅 发震时间:</td><td>{event['time']} (UTC+8)</td></tr>
        <tr><td>📍 震中位置:</td><td>{event.get('location', '未知')}</td></tr>
        <tr><td>🎯 震中坐标:</td><td>已隐藏 (规范要求)</td></tr>
        <tr><td>📊 震级:</td><td><b>M{event['magnitude']}</b></td></tr>
        <tr><td>📏 深度:</td><td>{event['depth']} km</td></tr>
        <tr><td>📏 震中距:</td><td>{distance:.1f} km</td></tr>
        <tr><td>📈 仪器最大烈度:</td>
            <td><b style="background:{color};padding:4px 12px;border-radius:4px;font-size:1.2em">{roman}度</b></td></tr>
        <tr><td>📈 预估本地烈度:</td><td><b style="color:red;font-size:1.2em">{intensity}度</b></td></tr>
        </table>
        <hr>
        <p style="color:#888;font-size:11px">
        说明: 本信息基于国家地震科学数据中心仪器烈度速报数据计算，仅供参考。<br>
        仪器烈度转换为CSIS烈度阶级时采用"四舍五入"规则，使用罗马数字表示。<br>
        本信息不构成官方预警，请以中国地震局正式发布为准。
        </p>
        </body></html>
        """
        send_email_alert(subject, html)

    def _send_dingtalk_v2(self, event, intensity, distance, epicenter_intensity):
        roman_map = {5: 'V', 6: 'VI', 7: 'VII', 8: 'VIII', 9: 'IX', 10: 'X', 11: 'XI', 12: 'XII'}
        roman = roman_map.get(epicenter_intensity, str(epicenter_intensity))

        webhook = f"https://oapi.dingtalk.com/robot/send?access_token={self.dingtalk_token}"
        markdown_text = (
            f"## ⚠️ 地震预警\n\n"
            f"- **发震时刻**: {event['time']}\n"
            f"- **震中位置**: {event.get('location', '未知')}\n"
            f"- **震级**: M{event['magnitude']}\n"
            f"- **深度**: {event['depth']}km\n"
            f"- **仪器最大烈度**: {roman}度\n"
            f"- **震中距**: {distance:.0f}km\n"
            f"- **预估本地烈度**: {intensity}度\n\n"
            f"---\n"
            f"数据来源: 国家地震科学数据中心\n"
            f"*本信息仅供参考，不构成官方预警*"
        )

        try:
            resp = requests.post(
                webhook,
                json={"msgtype": "markdown", "markdown": {
                    "title": f"地震预警 M{event['magnitude']}",
                    "text": markdown_text
                }},
                timeout=5
            )
            logger.info(f"📱 钉钉告警已发送")
        except Exception as e:
            logger.error(f"❌ 钉钉发送失败: {e}")


# ==================== 更新主程序 ====================
def cenc_monitor_v2(fetcher, calculator, dispatcher, interval=30):

        import time
        from datetime import datetime

        # 初始化
        initial_id = fetcher.get_last_id()
        if initial_id:
            fetcher.last_id = initial_id
            logger.info(f"📡 CENC 初始化: 最新事件ID = {initial_id}")

        # ===== 首次启动：拉取所有事件并强制展示 =====
        logger.info("📡 首次启动：拉取所有历史事件...")
        all_events = fetcher._fetch_event_list(minutes=None)
        if all_events:
            logger.info(f"📡 首次拉取到 {len(all_events)} 个事件")
            logger.info("=" * 60)
            for ev in all_events:
                intensity, distance, base_intensity = calculator.calculate(
                    ev['magnitude'], ev['depth'],
                    ev['lat'], ev['lng'],
                    ev.get('max_intensity', 0),
                    ev.get('instrumental_intensity', 0)
                )

                # ===== 强制打印所有事件详情 =====
                logger.info(f"📊 事件: {ev['time']}")
                logger.info(f"   震级: M{ev['magnitude']}  深度: {ev['depth']}km")
                logger.info(f"   位置: {ev['location']}")
                logger.info(f"   震中仪器烈度: {ev.get('max_intensity', 0)}度")
                logger.info(f"   震中距: {distance:.1f}km  预估本地烈度: {intensity}度")
                logger.info(f"   {'🚨 高烈度告警!' if intensity >= 4 else '✅ 低烈度(不触发告警)'}")
                logger.info("-" * 40)

                # 只有烈度 >= 4 才真正推送告警
                if intensity >= 4:
                    dispatcher.dispatch(ev, intensity, distance, ev.get('max_intensity', 0))

            logger.info("=" * 60)
        else:
            logger.info("📡 首次拉取：无事件")
        # ==============================================

        # ===== 后续轮询：正常逻辑 =====
        first_run = True
        while True:
            try:
                if first_run:
                    logger.info("📡 进入常规轮询模式（最近5分钟）")
                    first_run = False

                current_id = fetcher.get_last_id()

                if current_id and current_id != fetcher.last_id:
                    logger.info(f"🆕 检测到新事件! ID: {current_id}")
                    events = fetcher._fetch_event_list(minutes=5)

                    for ev in events:
                        intensity, distance, _ = calculator.calculate(
                            ev['magnitude'], ev['depth'],
                            ev['lat'], ev['lng'],
                            ev.get('max_intensity', 0),
                            ev.get('instrumental_intensity', 0)
                        )
                        # 常规模式：烈度 >= 4 才推送
                        if intensity >= 4:
                            dispatcher.dispatch(ev, intensity, distance, ev.get('max_intensity', 0))
                        else:
                            logger.info(
                                f"📊 {ev['time']} M{ev['magnitude']} {ev['location']} 本地烈度{intensity}度(不触发)")

                    fetcher.last_id = current_id
                else:
                    logger.debug(f"📡 轮询: 无新事件")

            except Exception as e:
                logger.error(f"❌ CENC 监听错误: {e}")

            time.sleep(interval)


# ==================== CENC 监听线程 ====================
def cenc_monitor(fetcher, calculator, dispatcher, interval=30):
    """
    CENC 实时监听线程
    首次启动拉取所有事件，之后只取最近5分钟的新事件
    """
    import time
    from datetime import datetime

    # 初始化：获取当前最新ID
    initial_id = fetcher.get_last_id()
    if initial_id:
        fetcher.last_id = initial_id
        logger.info(f"📡 CENC 初始化: 最新事件ID = {initial_id}")

    # ===== 首次启动：拉取所有事件（不带时间过滤） =====
    logger.info("📡 首次启动：拉取所有历史事件...")
    all_events = fetcher._fetch_event_list(minutes=None)  # None = 不过滤时间
    if all_events:
        logger.info(f"📡 首次拉取到 {len(all_events)} 个事件")
        for ev in all_events:
            intensity, distance, _ = calculator.calculate(
                ev['magnitude'], ev['depth'],
                ev['lat'], ev['lng'],
                ev.get('max_intensity', 0),
                ev.get('instrumental_intensity', 0)
            )
            dispatcher.dispatch(ev, intensity, distance, ev.get('max_intensity', 0))
    else:
        logger.info("📡 首次拉取：无事件")
    # ==================================================

    # ===== 后续轮询：只取最近5分钟 =====
    first_run = True
    while True:
        try:
            if first_run:
                logger.info("📡 进入常规轮询模式（最近5分钟）")
                first_run = False

            current_id = fetcher.get_last_id()

            if current_id and current_id != fetcher.last_id:
                logger.info(f"🆕 检测到新事件! ID: {current_id}")

                # 只拉取最近5分钟的事件
                events = fetcher._fetch_event_list(minutes=5)

                for ev in events:
                    intensity, distance, _ = calculator.calculate(
                        ev['magnitude'], ev['depth'],
                        ev['lat'], ev['lng'],
                        ev.get('max_intensity', 0),
                        ev.get('instrumental_intensity', 0)
                    )
                    dispatcher.dispatch(ev, intensity, distance, ev.get('max_intensity', 0))

                fetcher.last_id = current_id
            else:
                logger.debug(f"📡 轮询: 无新事件 (当前ID: {current_id})")

        except Exception as e:
            logger.error(f"❌ CENC 监听错误: {e}")

        time.sleep(interval)


# ==================== 主程序 ====================
def main():
    logger.info("🌍 地震预警服务器 v7.0 + CENC 启动")

    db = EarthquakeDatabase()
    logger.info(f"📂 数据库就绪，历史事件: {db.get_event_count()}")

    # MQTT 占位包装
    mqtt_wrapper = type('MQTTWrapper', (), {'publish': lambda self, t, p: None})()
    event_mgr = EventManager(db, mqtt_wrapper)
    mqtt_handler = MQTTHandler(event_mgr)
    mqtt_wrapper.publish = mqtt_handler.publish

    if not mqtt_handler.connect():
        logger.error("❌ MQTT 不可用，退出")
        return

        # 新代码（替换）：
    fetcher = CENCFetcherV2()  # 使用新 API 适配器
    calculator = IntensityCalculatorV2(STATION_LAT, STATION_LNG)  # 使用增强版烈度计算
    dispatcher = AlertDispatcherV2(mqtt_handler, DINGTALK_TOKEN)  # 使用合规告警分发

    threading.Thread(
        target=cenc_monitor_v2,  # 使用新的监听函数
        args=(fetcher, calculator, dispatcher, CENC_FETCH_INTERVAL),
        daemon=True
    ).start()
    logger.info("✅ CENC 实时目录监听已启动 (WebSocket 优先)")


    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("🛑 服务器关闭")
    finally:
        mqtt_handler.disconnect()


if __name__ == "__main__":
    main()
