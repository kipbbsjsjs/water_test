/* ============================================================
   超声波水位检测台  v1.0 (SSE推送)
   ------------------------------------------------------------
   用途: 独立测试HC-SR04水位检测, 网页实时查看距离/水位/波形/事件
   接线: HC-SR04
           VCC -> 5V(VIN)
           Trig-> GPIO25
           Echo-> GPIO33 (必须经1kΩ+2kΩ分压至3.3V)
           GND -> GND
         HC-SR501 PIR
           VCC -> 5V(VIN)
           OUT -> GPIO16(RX2丝印)
           GND -> GND
         OLED 0.96 I2C
           VDD -> 3.3V (禁止5V!)
           GND -> GND
           SDA -> GPIO21
           SCL -> GPIO22
   传输: SSE长连接, 250ms采样周期, 端到端延迟约250ms
   滤波: 3次采样取中位数, 剔除异常值
   注意: 上电即可用, 无需预热; OLED开机秒亮, 不等WiFi
   ============================================================ */
#include <WiFi.h>
#include <WebServer.h>

// ---------------- 引脚 ----------------
#define PIN_TRIG 25   // 超声波Trig输出
#define PIN_ECHO 33   // 超声波Echo输入(经分压)
#define PIN_PIR  16   // PIR输出, 高电平=有人(RX2丝印)

// ---------------- OLED ----------------
#define USE_OLED 1     // 1=启用OLED(需安装U8g2库)
#if USE_OLED
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#endif

// ---------------- WiFi (STA) ----------------
const char* WIFI_SSID = "YOUR_SSID";      // 烧录前改成自己的热点名
const char* WIFI_PASS = "YOUR_PASS";      // 烧录前改成自己的热点密码

// ---------------- 水槽参数 ----------------
const float TANK_DEPTH_CM = 20.0f;   // 水槽深度(传感器到槽底距离)
const float TANK_AREA_CM2 = 150.0f;  // 水槽截面积(用于水量换算)
const float YUAN_PER_L    = 5.0f;     // 模拟费率: 元/升

// ---------------- 采样/记录 ----------------
const unsigned long SAMPLE_MS = 250;   // 采样周期, 决定推送频率
const uint8_t HIST_N = 96;             // 波形环形缓冲 96格
const uint8_t LOG_N  = 10;             // 事件日志上限

// ---------------- 业务变量 ----------------
float distCm = -1.0f;             // 最新测得距离(cm), -1=失败
float waterCm = 0.0f;             // 当前水位深度(cm)
float waterStartCm = 0.0f;        // 起始水位(基准)
float peakHighCm = 0.0f;          // 历史最高水位
float peakLowCm = 0.0f;           // 历史最低水位
unsigned long sampleCount = 0;    // 累计采样次数
unsigned long failCount = 0;      // 失败次数(超时/异常)
bool pirState = false;            // PIR状态: true=有人
uint8_t hist[HIST_N];             // 水位百分比环形缓冲 0-100
uint8_t histIdx = 0;
String logs[LOG_N];               // 事件日志, 新的在前
uint8_t logCount = 0;
float lastPushedCm = -1.0f;       // 上次推送水位, 用于检测变化事件

WebServer server(80);

void pushLog(const String& s) {
  if (logCount < LOG_N) logCount++;
  for (uint8_t i = logCount - 1; i > 0; i--) logs[i] = logs[i - 1];
  logs[0] = s;
}

// ---------------- 单次测距 ----------------
float readDistanceOnce() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);  // 25ms超时
  if (us == 0) return -1.0f;
  float d = us * 0.0343f / 2.0f;
  if (d <= 0 || d > 400) return -1.0f;   // 量程过滤
  return d;
}

// ---------------- 中位数滤波采样 ----------------
float sampleDistance() {
  float d[3]; uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++) {
    float x = readDistanceOnce();
    if (x > 0) d[n++] = x;
    delayMicroseconds(500);
  }
  if (n == 0) return -1.0f;
  // 冒泡排序取中位数
  for (uint8_t i = 1; i < n; i++)
    for (uint8_t j = i; j > 0 && d[j] < d[j - 1]; j--) {
      float t = d[j]; d[j] = d[j - 1]; d[j - 1] = t;
    }
  return d[n / 2];
}

// ---------------- 水量计算 ----------------
float calcVolumeL() {
  // 当前水位相对起始水位的差值 => 用水量(下降视为消耗)
  float dv = waterStartCm - waterCm;
  if (dv < 0) dv = 0;
  return dv * TANK_AREA_CM2 / 1000.0f;   // cm³ -> L
}
float calcMoney() { return calcVolumeL() * YUAN_PER_L; }

// ---------------- 采样任务 ----------------
bool sampleTask() {
  static unsigned long lastS = 0;
  if (millis() - lastS < SAMPLE_MS) return false;
  lastS = millis();

  float dist = sampleDistance();
  sampleCount++;
  pirState = (digitalRead(PIN_PIR) == HIGH);    // 同步读取PIR
  if (dist < 0) {
    failCount++;
    return false;                          // 失败时不更新水位, 不推送
  }
  distCm = dist;
  float lv = TANK_DEPTH_CM - dist;
  waterCm = constrain(lv, 0.0f, TANK_DEPTH_CM);

  // 峰值记录
  if (waterCm > peakHighCm) peakHighCm = waterCm;
  if (peakLowCm == 0 || (waterCm > 0 && waterCm < peakLowCm)) peakLowCm = waterCm;

  // 波形缓冲: 水位百分比
  uint8_t pct = (uint8_t)(waterCm / TANK_DEPTH_CM * 100.0f);
  hist[histIdx] = pct;
  histIdx = (histIdx + 1) % HIST_N;

  // 水位显著变化事件(>1.0cm)
  if (lastPushedCm >= 0 && fabs(waterCm - lastPushedCm) >= 1.0f) {
    char buf[64];
    snprintf(buf, sizeof(buf), "[%6.1fs] 水位 %.1f -> %.1f cm",
             millis() / 1000.0f, lastPushedCm, waterCm);
    pushLog(String(buf));
    Serial.println(buf);
  }
  lastPushedCm = waterCm;
  return true;
}

// ---------------- OLED 本地显示 ----------------
#if USE_OLED
void oledInit() {
  if (oled.begin()) {
    Serial.println("[OLED] 已启用 (U8g2)");
    oled.clearBuffer();
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 12, "启动中...");
    oled.sendBuffer();
  } else {
    Serial.println("[OLED] 初始化失败, 检查接线/地址");
  }
}

void oledRefresh() {
  char buf[40];
  oled.clearBuffer();
  oled.setFont(u8g2_font_wqy12_t_gb2312);   // 全程用同一字体, 不切换
  // 行1: IP
  String ipStr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("WiFi断开");
  oled.drawUTF8(0, 12, ipStr.c_str());
  // 行2: PIR状态
  oled.drawUTF8(0, 28, pirState ? "状态: 有人" : "状态: 无人");
  // 行3: 距离
  if (distCm > 0) {
    snprintf(buf, sizeof(buf), "距离: %.1f 厘米", distCm);
    oled.drawUTF8(0, 44, buf);
  } else {
    oled.drawUTF8(0, 44, "距离: 测距失败");
  }
  oled.sendBuffer();
}
#endif

// ---------------- 状态JSON ----------------
String jsonStatus() {
  String j;
  j.reserve(512);
  j = "{";
  j += "\"dist\":" + String(distCm, 2) + ",";
  j += "\"wl\":" + String(waterCm, 2) + ",";
  j += "\"depth\":" + String(TANK_DEPTH_CM, 1) + ",";
  j += "\"vol\":" + String(calcVolumeL(), 2) + ",";
  j += "\"money\":" + String(calcMoney(), 2) + ",";
  j += "\"ph\":" + String(peakHighCm, 2) + ",";
  j += "\"pl\":" + String(peakLowCm, 2) + ",";
  j += "\"sc\":" + String(sampleCount) + ",";
  j += "\"fc\":" + String(failCount) + ",";
  j += "\"up\":" + String(millis() / 1000) + ",";
  j += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  j += "\"log\":[";
  for (uint8_t i = 0; i < logCount; i++) { if (i) j += ","; j += "\"" + logs[i] + "\""; }
  j += "],\"his\":[";
  for (uint8_t i = 0; i < HIST_N; i++) { if (i) j += ","; j += String(hist[(histIdx + i) % HIST_N]); }
  j += "]}";
  return j;
}

// ---------------- SSE 长连接推送 ----------------
void handleEvents() {
  WiFiClient client = server.client();
  client.setNoDelay(true);
  server.sendHeader("Content-Type", "text/event-stream");
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Connection", "keep-alive");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendContent(": stream open\n\n");
  // 立即推送一次, 避免页面初始空白
  server.sendContent("data: " + jsonStatus() + "\n\n");
  while (client.connected()) {
    if (sampleTask()) {
      server.sendContent("data: " + jsonStatus() + "\n\n");
    }
    delay(1);
  }
}

static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>超声波水位检测台</title>
<style>
:root{--bg:#060a10;--panel:#0c1320;--panel2:#0a101b;--line:#1c2940;--cyan:#2dd4e8;--green:#34d399;
--amber:#fbbf24;--red:#f87171;--blue:#60a5fa;--text:#dbe7f4;--dim:#5b6b82;--mono:Consolas,'Courier New',monospace}
*{margin:0;padding:0;box-sizing:border-box}
body{background:var(--bg);color:var(--text);font-family:var(--mono);min-height:100vh;
background-image:linear-gradient(rgba(45,212,232,.03) 1px,transparent 1px),linear-gradient(90deg,rgba(45,212,232,.03) 1px,transparent 1px);
background-size:28px 28px}
body::after{content:'';position:fixed;inset:0;pointer-events:none;background:repeating-linear-gradient(0deg,rgba(0,0,0,.12) 0 1px,transparent 1px 3px)}
.wrap{max-width:680px;margin:0 auto;padding:18px 14px 40px}
header{display:flex;justify-content:space-between;align-items:center;padding:10px 0 16px;border-bottom:1px solid var(--line);margin-bottom:16px}
.brand{display:flex;gap:10px;align-items:center}
.brand .logo{width:34px;height:34px;border:1px solid var(--cyan);position:relative;display:grid;place-items:center;color:var(--cyan);font-size:16px}
.brand .logo::after{content:'';position:absolute;inset:3px;border:1px solid rgba(45,212,232,.3)}
.brand h1{font-size:15px;font-weight:600;letter-spacing:2px}
.brand small{display:block;font-size:10px;color:var(--dim);letter-spacing:3px;font-weight:400}
#badge{padding:7px 16px;font-size:13px;letter-spacing:2px;border:1px solid;border-radius:2px;position:relative}
#badge::before,#badge::after{content:'';position:absolute;width:6px;height:6px;border:1px solid currentColor}
#badge::before{top:-1px;left:-1px;border-right:none;border-bottom:none}
#badge::after{bottom:-1px;right:-1px;border-left:none;border-top:none}
.stok{color:var(--green);border-color:var(--green)}
.sterr{color:var(--red);border-color:var(--red);animation:bl .6s infinite}
@keyframes bl{50%{opacity:.3}}
.card{background:var(--panel);border:1px solid var(--line);padding:14px;margin-bottom:10px;position:relative}
.card::before{content:'';position:absolute;top:-1px;left:-1px;width:7px;height:7px;border-top:1px solid var(--cyan);border-left:1px solid var(--cyan)}
.card h3{font-size:11px;color:var(--dim);letter-spacing:3px;font-weight:400;margin-bottom:12px;display:flex;justify-content:space-between}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.big{font-size:38px;line-height:1.1;text-shadow:0 0 14px rgba(45,212,232,.4)}
.big.ok{color:var(--cyan)}
.big.err{color:var(--red);text-shadow:0 0 14px rgba(248,113,113,.5);animation:bl .6s infinite}
.big small{font-size:13px;color:var(--dim);text-shadow:none;margin-left:6px}
.row{display:flex;justify-content:space-between;align-items:baseline;padding:7px 0;border-bottom:1px dashed rgba(28,41,64,.8);font-size:13px}
.row:last-child{border:none}
.row b{font-weight:400;color:var(--dim)}
.row .v{color:var(--text)} .row .v.money{color:var(--cyan);font-size:17px;text-shadow:0 0 10px rgba(45,212,232,.35)}
.row .v.warn{color:var(--amber)}

/* 水位可视化 */
.tank{display:flex;gap:14px;align-items:stretch;margin-top:6px}
.tankvis{width:48px;height:200px;background:var(--panel2);border:1px solid var(--line);position:relative;overflow:hidden;flex:none}
.tankvis::before{content:'';position:absolute;left:0;right:0;bottom:0;height:0%;
background:linear-gradient(180deg,rgba(45,212,232,.85),rgba(45,212,232,.45));
border-top:2px solid var(--cyan);box-shadow:0 -2px 12px rgba(45,212,232,.6);transition:height .25s}
.tankvis::after{content:'';position:absolute;left:0;right:0;top:0;height:8px;
background:repeating-linear-gradient(45deg,rgba(96,165,250,.15) 0 4px,transparent 4px 8px)}
.tkmarks{position:absolute;inset:0;pointer-events:none}
.tkmarks span{position:absolute;left:100%;font-size:9px;color:var(--dim);transform:translateY(-50%);padding-left:4px;white-space:nowrap}
.tkmarks span::before{content:'';position:absolute;right:100%;top:50%;width:4px;height:1px;background:var(--dim)}
.tankinfo{flex:1;display:flex;flex-direction:column;justify-content:space-between}
.tankinfo .row{padding:5px 0}

.box{display:flex;gap:2px;height:60px;align-items:flex-end;margin-top:6px}
.box i{flex:1;background:#13202f;border:1px solid var(--line);min-height:1px;transition:height .25s,background .25s}
.box i.lv0{background:rgba(45,212,232,.15)}
.box i.lv1{background:rgba(45,212,232,.4)}
.box i.lv2{background:rgba(45,212,232,.7);box-shadow:0 0 4px rgba(45,212,232,.5)}
.box i.lv3{background:var(--cyan);box-shadow:0 0 6px rgba(45,212,232,.8)}
.legend{display:flex;justify-content:space-between;font-size:11px;color:var(--dim);margin-top:6px}
#log div{padding:6px 0;border-bottom:1px dashed rgba(28,41,64,.8);font-size:12px;color:var(--dim)}
#log div:last-child{border:none}
#log .empty{color:var(--dim);opacity:.6}
footer{margin-top:16px;padding-top:12px;border-top:1px solid var(--line);display:flex;justify-content:space-between;font-size:10.5px;color:var(--dim);letter-spacing:1px}
.dot{display:inline-block;width:6px;height:6px;border-radius:50%;background:var(--green);margin-right:5px;animation:pl 2s infinite}
@keyframes pl{50%{opacity:.4}}
.reveal{opacity:0;transform:translateY(8px);animation:rv .5s forwards}
@keyframes rv{to{opacity:1;transform:none}}
@media(max-width:520px){.grid{grid-template-columns:1fr}.tankvis{height:160px}}
</style></head><body><div class="wrap">
<header class="reveal">
  <div class="brand"><div class="logo">≈</div><h1>超声波水位检测台<small>HC-SR04 TESTBENCH</small></h1></div>
  <div id="badge" class="stok">正常</div>
</header>

<div class="grid">
<div class="card reveal" style="animation-delay:.08s">
  <h3>实时测距 <span id="distTag">--</span></h3>
  <div class="big ok" id="bigDist">--<small>cm</small></div>
  <div class="row"><b>当前水位</b><span class="v" id="wl">-- cm</span></div>
  <div class="row"><b>用水量</b><span class="v" id="vol">0.00 L</span></div>
  <div class="row"><b>模拟金额</b><span class="v money" id="money">¥ 0.00</span></div>
</div>

<div class="card reveal" style="animation-delay:.14s">
  <h3>水槽视图 <span>槽深 <span id="depth">20.0</span>cm</span></h3>
  <div class="tank">
    <div class="tankvis" id="tank">
      <div class="tkmarks">
        <span style="top:0%">100%</span>
        <span style="top:25%">75%</span>
        <span style="top:50%">50%</span>
        <span style="top:75%">25%</span>
        <span style="top:100%">0%</span>
      </div>
    </div>
    <div class="tankinfo">
      <div class="row"><b>历史最高</b><span class="v" id="ph">--</span></div>
      <div class="row"><b>历史最低</b><span class="v" id="pl">--</span></div>
      <div class="row"><b>水位占比</b><span class="v" id="pct">--</span></div>
      <div class="row"><b>采样次数</b><span class="v" id="sc">0</span></div>
      <div class="row"><b>失败次数</b><span class="v" id="fc">0</span></div>
    </div>
  </div>
</div>
</div>

<div class="card reveal" style="animation-delay:.2s">
  <h3>水位波形 <span id="wtag">近 24 秒</span></h3>
  <div class="box" id="box"></div>
  <div class="legend"><span>0%</span><span>每格 250ms</span><span>100%</span></div>
</div>

<div class="card reveal" style="animation-delay:.26s">
  <h3>事件日志 <span>最新在上 · 最多10条</span></h3>
  <div id="log"><div class="empty">暂无事件</div></div>
</div>

<footer class="reveal" style="animation-delay:.3s">
  <span><i class="dot" id="conn"></i><span id="connTxt">已连接</span></span>
  <span id="upTxt">--</span><span>WATER TEST v1.0 / SSE</span>
</footer>
</div>

<script>
const $=id=>document.getElementById(id);
const box=$('box');
for(let i=0;i<96;i++){box.appendChild(document.createElement('i'))}
function cls(pct){
  if(pct>=75)return'lv3';if(pct>=50)return'lv2';if(pct>=25)return'lv1';if(pct>0)return'lv0';return'';
}
function render(d){
  const ok = d.dist>=0;
  const bd=$('bigDist');
  bd.className='big '+(ok?'ok':'err');
  bd.innerHTML = ok? d.dist.toFixed(2)+'<small>cm</small>' : 'ERR<small>超时</small>';
  $('distTag').textContent = ok? 'OK' : '⚠ 超时';
  const badge=$('badge');
  badge.className = ok?'stok':'sterr';
  badge.textContent = ok?'正常':'异常';
  $('wl').textContent = d.wl.toFixed(2)+' cm';
  $('vol').textContent = d.vol.toFixed(2)+' L';
  $('money').textContent = '¥ '+d.money.toFixed(2);
  $('depth').textContent = d.depth.toFixed(1);
  $('ph').textContent = d.ph>0? d.ph.toFixed(2)+' cm':'--';
  $('pl').textContent = d.pl>0? d.pl.toFixed(2)+' cm':'--';
  const pct = d.depth>0? (d.wl/d.depth*100):0;
  $('pct').textContent = pct.toFixed(1)+'%';
  $('sc').textContent = d.sc+' 次';
  const fcEl=$('fc');fcEl.textContent=d.fc+' 次';
  fcEl.className='v'+(d.fc>0?' warn':'');
  // 水槽填充: 伪元素height无法直接JS修改, 用style标签覆盖
  const ex=document.getElementById('tkStyle')||document.createElement('style');
  ex.id='tkStyle';ex.textContent='.tankvis::before{height:'+pct.toFixed(1)+'%}';
  if(!ex.parentNode)document.head.appendChild(ex);
  // 波形
  const bars=box.children;
  for(let i=0;i<96;i++){const p=d.his[i];bars[i].style.height=p+'%';bars[i].className=p>0?cls(p):''}
  // 日志
  const lg=$('log');
  if(d.log.length){lg.innerHTML=d.log.map(s=>'<div>'+s+'</div>').join('')}
  $('upTxt').textContent='开机 '+d.up+'s / '+d.rssi+'dBm';
  $('connTxt').textContent='已连接';$('conn').style.background='var(--green)';
}
if(location.protocol.startsWith('http'))$('upTxt').textContent=location.host;
const es=new EventSource('/events');
es.onmessage=e=>{try{render(JSON.parse(e.data));}catch(_){}};
es.onerror=()=>{$('connTxt').textContent='重连中';$('conn').style.background='var(--amber)';};
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ---------------- 初始化 ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_PIR, INPUT);
  digitalWrite(PIN_TRIG, LOW);

#if USE_OLED
  oledInit();
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] 正在连接 %s", WIFI_SSID);
  unsigned long wifiT0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiT0 < 20000) {
    delay(250); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] 已连接，IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] 暂未连上，后台自动重连中");
  }
  server.on("/", handleRoot);
  server.on("/events", handleEvents);
  server.enableCORS(true);
  server.begin();

  // 上电预采样一次, 给网页首推有数据
  float dist = sampleDistance();
  if (dist > 0) {
    distCm = dist;
    waterCm = constrain(TANK_DEPTH_CM - dist, 0.0f, TANK_DEPTH_CM);
    waterStartCm = waterCm;     // 以开机水位作为基准
    peakHighCm = waterCm;
    peakLowCm = waterCm;
    Serial.printf("[Init] 初始距离 %.2f cm / 水位 %.2f cm\n", distCm, waterCm);
  } else {
    Serial.println("[Init] 首次采样失败, 检查接线");
  }

  Serial.println("========================================");
  Serial.println("  超声波水位检测台 v1.0 (SSE)");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("  网页: http://%s\n", WiFi.localIP().toString().c_str());
  }
  Serial.println("========================================");
}

void loop() {
  server.handleClient();
  sampleTask();   // 无SSE客户端时仍保持采样, 峰值/日志不丢
  static unsigned long lastWifiChk = 0;
  static bool wasConnected = false;
  if (millis() - lastWifiChk >= 5000) {
    lastWifiChk = millis();
    bool ok = (WiFi.status() == WL_CONNECTED);
    if (ok && !wasConnected) Serial.printf("[WiFi] 已连接，IP: %s\n", WiFi.localIP().toString().c_str());
    if (!ok && wasConnected) Serial.println("[WiFi] 断开，自动重连中...");
    wasConnected = ok;
  }
#if USE_OLED
  static unsigned long lastOled = 0;
  if (millis() - lastOled >= 200) { lastOled = millis(); oledRefresh(); }
#endif
}
