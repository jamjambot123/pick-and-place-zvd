/*****************************************************************************
 *  [프로젝트명] ESP32-S3 기반 ZVD 입력 성형 및 역기구학 적용
 *              진공 픽앤플레이스 로봇팔 펌웨어 (웹 GUI 추가판)
 *
 *  [MCU]       ESP32-S3 DevKitC (듀얼 코어, FreeRTOS, Wi-Fi AP)
 *  [모터드라이버] TB6600 x3 (24V SMPS, 3.3V 로직 호환 옵토커플러 내장)
 *  [센서]       MPU6050 가속도/자이로 (I2C, 3.3V)
 *  [진공]       릴레이 모듈 2채널 (5V 전원, 3.3V 로직 트리거)
 *
 *  [웹 제어]
 *    - Wi-Fi AP: "RobotArm_ZVD_WiFi" (비밀번호: 12345678)
 *    - 웹 주소: http://192.168.4.1
 *    - 기능: 실시간 상태 모니터링, 좌표 동적 설정, 사이클 실행
 *
 *  Copyright (c) 2026 기말작품 프로젝트
 *****************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ★ 웹 서버 추가용 라이브러리
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>  // 플래시 메모리 저장용

// ═══ 구조체/상수 전방 선언 (모든 함수보다 앞에 위치해야 컴파일됨) ═══
#define ZVD_NUM_IMPULSES 3
struct CartesianPoint { float x; float y; float z; };
struct JointAngles { float base_deg; float shoulder_deg; float elbow_deg; bool valid; };
struct MotionCommand { int32_t target_steps_base; int32_t target_steps_shoulder; int32_t target_steps_elbow; uint32_t step_interval_us; bool use_zvd; bool completed; };
struct ZVD_Impulse { float amplitude[ZVD_NUM_IMPULSES]; float delay_ms[ZVD_NUM_IMPULSES]; };
struct MPU6050_Data { float ax, ay, az; float gx, gy, gz; };

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 1: 하드웨어 핀맵 선언 (부팅 간섭 없는 안전 GPIO만 사용)
// ═══════════════════════════════════════════════════════════════════════════

#define MOTOR1_BASE_STEP      4    // 베이스 회전축 STEP (PUL+)
#define MOTOR1_BASE_DIR       5    // 베이스 회전축 DIR  (DIR+)
#define MOTOR1_BASE_ENA       8    // 베이스 ENA  (LOW=활성, HIGH=비활성)
#define MOTOR2_SHOULDER_STEP  15   // 숄더(하부 암, 몸체쪽) STEP — GPIO 15/16 모터
#define MOTOR2_SHOULDER_DIR   16   // 숄더(하부 암, 몸체쪽) DIR
#define MOTOR2_SHOULDER_ENA   10   // 숄더 ENA
#define MOTOR3_ELBOW_STEP     6    // 엘보(상부 암, 끝단쪽) STEP — GPIO 6/7 모터
#define MOTOR3_ELBOW_DIR      7    // 엘보(상부 암, 끝단쪽) DIR
#define MOTOR3_ELBOW_ENA      9    // 엘보 ENA

#define LIMIT_BASE            1    // 베이스 영점 스위치
#define LIMIT_SHOULDER        47   // 숄더 영점 스위치 (물리적 스위치 교환 반영)
#define LIMIT_ELBOW           2    // 엘보 영점 스위치 (물리적 스위치 교환 반영)

// 스위치 논리 — 핀 교환 후 실제 연결된 스위치 타입 반영
#define LIMIT_BASE_ACTIVE     HIGH // NC
#define LIMIT_SHOULDER_ACTIVE LOW  // NO (47번 핀에 연결된 스위치)
#define LIMIT_ELBOW_ACTIVE    HIGH // NC (2번 핀에 연결된 스위치)

#define RELAY_VACUUM_PUMP     21   // 진공 펌프 릴레이 제어 핀
// 릴레이 작동 로직 (Low-Level Trigger: LOW=켜짐, HIGH=오픈드레인 꺼짐)
#define RELAY_ON              LOW
#define RELAY_OFF             HIGH

#define MPU6050_SDA           17   // I2C 데이터 라인
#define MPU6050_SCL           18   // I2C 클럭 라인
#define MPU6050_ADDR          0x68 // 기본 I2C 주소

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 2: 로봇팔 물리 상수 및 스텝 설정
// ═══════════════════════════════════════════════════════════════════════════

#define LOW_SHANK_LENGTH      120.0f
#define HIGH_SHANK_LENGTH     120.0f
#define END_EFFECTOR_OFFSET   54.0f
#define BASE_HEIGHT           0.0f

#define STEPS_PER_REV         200
#define MICROSTEPS            8

#define MOTOR_GEAR_TEETH      20.0f
#define MAIN_GEAR_TEETH_BASE  90.0f  // 원작 20sffactory 벨트 버전 기어비 (90T)
#define MAIN_GEAR_TEETH_SHOULDER 90.0f
#define MAIN_GEAR_TEETH_ELBOW 90.0f

#define GEAR_RATIO_BASE       (MAIN_GEAR_TEETH_BASE / MOTOR_GEAR_TEETH)
#define GEAR_RATIO_SHOULDER   (MAIN_GEAR_TEETH_SHOULDER / MOTOR_GEAR_TEETH)
#define GEAR_RATIO_ELBOW      (MAIN_GEAR_TEETH_ELBOW / MOTOR_GEAR_TEETH)

#define STEPS_PER_DEG_BASE    ((float)(STEPS_PER_REV * MICROSTEPS) * GEAR_RATIO_BASE / 360.0f)
#define STEPS_PER_DEG_SHOULDER ((float)(STEPS_PER_REV * MICROSTEPS) * GEAR_RATIO_SHOULDER / 360.0f)
#define STEPS_PER_DEG_ELBOW   ((float)(STEPS_PER_REV * MICROSTEPS) * GEAR_RATIO_ELBOW / 360.0f)

#define HOMING_SPEED_US       2000
#define HOMING_BACKOFF_STEPS  500
#define HOME_DWELL_MS         200
#define HOMING_APPROACH_SPEED_US  4000

// 호밍 방향 (LOW/HIGH = DIR 핀 레벨, 스위치 쪽으로 가는 방향)
#define HOMING_DIR_BASE       LOW
#define HOMING_DIR_SHOULDER   HIGH  // GPIO 15/16 모터의 스위치 접근 방향
#define HOMING_DIR_ELBOW      LOW   // GPIO 6/7 모터의 스위치 접근 방향

// 호밍 후 초기 자세까지 물리적으로 되돌아갈 스텝 수 (원작 v0.81 config.h)
#define HOME_OFFSET_BASE      3640  // Z_HOME_STEPS (베이스 회전)
#define HOME_OFFSET_SHOULDER  1900  // Y_HOME_STEPS (Lower Arm = 숄더, 몸체쪽)
#define HOME_OFFSET_ELBOW     1020  // X_HOME_STEPS (Upper Arm = 엘보우, 끝단쪽)

// 호밍 후 초기 자세에서의 각도 (원작: Lower=0°, Upper=90°, Rotate=0°)
#define HOME_INITIAL_DEG_BASE      0.0f
#define HOME_INITIAL_DEG_SHOULDER  0.0f   // 숄더 수평 = 0도
#define HOME_INITIAL_DEG_ELBOW     90.0f  // 엘보우 직각 = 90도

#define MIN_STEP_INTERVAL_US  100
#define MAX_STEP_INTERVAL_US  5000
#define ACCEL_STEPS           80

// 안전 제한
#define Z_MIN_SAFE            0.0f    // Z축 하한 (mm) - 0으로 설정 (하강 제한 없음)
#define CARTESIAN_INTERP_MM   5.0f    // 직선 보간 세그먼트 길이 (mm)

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 3: ZVD 입력 성형 파라미터
// ═══════════════════════════════════════════════════════════════════════════

#define DEFAULT_NATURAL_FREQ_HZ   8.0f
#define DEFAULT_DAMPING_RATIO     0.05f

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 4: 픽앤플레이스 티칭 설정 (관절 스텝 저장 방식)
// ═══════════════════════════════════════════════════════════════════════════

// 티칭된 위치 (관절 스텝 카운터로 저장 — IK 완전 불필요)
struct TeachPoint {
  int32_t steps_base;
  int32_t steps_shoulder;
  int32_t steps_elbow;
  bool saved;
};
TeachPoint pointA_up   = {0, 0, 0, false};  // A 위 (대기 높이)
TeachPoint pointA_down = {0, 0, 0, false};  // A 아래 (집는 높이)
TeachPoint pointB_up   = {0, 0, 0, false};  // B 위 (대기 높이)
TeachPoint pointB_down = {0, 0, 0, false};  // B 아래 (놓는 높이)
volatile bool webCmd_SaveAU = false;
volatile bool webCmd_SaveAD = false;
volatile bool webCmd_SaveBU = false;
volatile bool webCmd_SaveBD = false;
volatile bool webCmd_ContinuousCycle = false;
volatile int32_t cycleCount = 0;

Preferences prefs;

void saveOnePoint(const char* prefix, TeachPoint& pt) {
  char key[16];
  snprintf(key, sizeof(key), "%s_b", prefix); prefs.putInt(key, pt.steps_base);
  snprintf(key, sizeof(key), "%s_s", prefix); prefs.putInt(key, pt.steps_shoulder);
  snprintf(key, sizeof(key), "%s_e", prefix); prefs.putInt(key, pt.steps_elbow);
  snprintf(key, sizeof(key), "%s_ok", prefix); prefs.putBool(key, pt.saved);
}
void loadOnePoint(const char* prefix, TeachPoint& pt) {
  char key[16];
  snprintf(key, sizeof(key), "%s_b", prefix); pt.steps_base = prefs.getInt(key, 0);
  snprintf(key, sizeof(key), "%s_s", prefix); pt.steps_shoulder = prefs.getInt(key, 0);
  snprintf(key, sizeof(key), "%s_e", prefix); pt.steps_elbow = prefs.getInt(key, 0);
  snprintf(key, sizeof(key), "%s_ok", prefix); pt.saved = prefs.getBool(key, false);
}
void saveTeachPoints() {
  prefs.begin("teach", false);
  saveOnePoint("au", pointA_up); saveOnePoint("ad", pointA_down);
  saveOnePoint("bu", pointB_up); saveOnePoint("bd", pointB_down);
  prefs.end();
  Serial.println("[FLASH] 티칭 데이터 저장 완료");
}
void loadTeachPoints() {
  prefs.begin("teach", true);
  loadOnePoint("au", pointA_up); loadOnePoint("ad", pointA_down);
  loadOnePoint("bu", pointB_up); loadOnePoint("bd", pointB_down);
  prefs.end();
  Serial.printf("[FLASH] 로드: A위=%s A아래=%s B위=%s B아래=%s\n",
    pointA_up.saved?"OK":"X", pointA_down.saved?"OK":"X",
    pointB_up.saved?"OK":"X", pointB_down.saved?"OK":"X");
}

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 5: FreeRTOS & 제어 상태 상수
// ═══════════════════════════════════════════════════════════════════════════

#define TASK_CONTROL_STACK    8192
#define TASK_STEPPER_STACK    4096
#define TASK_CONTROL_PRIORITY 2
#define TASK_STEPPER_PRIORITY 24
#define MOTION_QUEUE_SIZE     8



enum PickPlaceState {
  STATE_HOMING = 0,
  STATE_IDLE,
  // Phase 1: A→B (집기)
  STATE_MOVE_TO_A,
  STATE_DESCEND_A,
  STATE_PICK,
  STATE_ASCEND_A,
  STATE_MOVE_TO_B,
  STATE_DESCEND_B,
  STATE_PLACE,
  STATE_ASCEND_B,
  // Phase 2: B→A (되가져오기)
  STATE_DESCEND_B2,   // B아래로 (다시 집기)
  STATE_PICK_B,       // 진공 흡착
  STATE_ASCEND_B2,    // B위로
  STATE_RETURN_A,     // A위로 이동
  STATE_DESCEND_A2,   // A아래로 (놓기)
  STATE_PLACE_A,      // 진공 해제
  STATE_ASCEND_A2     // A위로 → 1사이클 완료
};

volatile PickPlaceState currentState = STATE_IDLE;
String stateStrings[] = {"Homing...", "IDLE",
  "→A↑", "A↓", "Pick", "A↑", "→B↑", "B↓", "Place", "B↑",
  "B↓₂", "PickB", "B↑₂", "→A↑", "A↓₂", "PlaceA", "A↑₂"};

// 웹 서버 인스턴스 (포트 80)
WebServer server(80);

volatile int32_t currentSteps_base = 0;
volatile int32_t currentSteps_shoulder = 0;
volatile int32_t currentSteps_elbow = 0;
volatile bool motionComplete = true;
volatile bool webCmd_EStop = false;  // 비상정지 플래그

portMUX_TYPE stepsMux = portMUX_INITIALIZER_UNLOCKED;  // FK 읽기용 스핀락

// 웹 제어 트리거
volatile bool webCmd_Rehome = false;
volatile bool webCmd_TuneZVD = false;
volatile char webCmd_JogXYZ_Axis = 0;
volatile float webCmd_JogXYZ_Dist = 0.0f;

TaskHandle_t taskControlHandle = NULL;
TaskHandle_t taskStepperHandle = NULL;
QueueHandle_t motionQueue = NULL;
SemaphoreHandle_t motionDoneSem = NULL;

float zvd_natural_freq_hz = DEFAULT_NATURAL_FREQ_HZ;
float zvd_damping_ratio = DEFAULT_DAMPING_RATIO;
MPU6050_Data mpu_data = {0};
SemaphoreHandle_t mpuMutex = NULL;

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 6: 웹 GUI (HTML/CSS/JS) - PROGMEM 저장
// ═══════════════════════════════════════════════════════════════════════════

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ZVD 로봇팔 대시보드</title>
  <style>
    :root {
      --bg: #0f172a;
      --card-bg: rgba(30, 41, 59, 0.7);
      --primary: #06b6d4;
      --primary-hover: #0891b2;
      --accent: #f43f5e;
      --text: #f8fafc;
      --text-dim: #cbd5e1;
    }
    * { box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
    body { background-color: var(--bg); color: var(--text); margin: 0; padding: 20px;
           background-image: radial-gradient(circle at 10% 20%, rgba(6, 182, 212, 0.15) 0%, transparent 20%),
                             radial-gradient(circle at 90% 80%, rgba(244, 63, 94, 0.1) 0%, transparent 20%);
           min-height: 100vh; }
    h1 { text-align: center; font-weight: 300; letter-spacing: 2px; margin-bottom: 30px; }
    h1 span { font-weight: bold; color: var(--primary); }
    
    .container { max-width: 1000px; margin: 0 auto; display: grid; grid-template-columns: 1fr; gap: 20px; }
    @media (min-width: 768px) { .container { grid-template-columns: 1fr 1fr; } }
    
    .glass-card {
      background: var(--card-bg);
      backdrop-filter: blur(12px);
      border: 1px solid rgba(255,255,255,0.05);
      border-radius: 16px;
      padding: 24px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.3);
      transition: transform 0.3s ease;
    }
    .glass-card:hover { transform: translateY(-2px); }
    
    h2 { margin-top: 0; font-size: 1.2rem; color: var(--text-dim); border-bottom: 1px solid rgba(255,255,255,0.1); padding-bottom: 10px; }
    
    .status-badge {
      display: inline-block; padding: 8px 16px; border-radius: 20px;
      background: rgba(6, 182, 212, 0.2); border: 1px solid var(--primary);
      color: var(--primary); font-weight: bold; font-size: 1.1rem;
      text-align: center; width: 100%; margin-bottom: 20px;
    }
    .status-badge.active { background: rgba(244, 63, 94, 0.2); border-color: var(--accent); color: var(--accent); }
    
    .data-row { display: flex; justify-content: space-between; margin-bottom: 10px; font-size: 1.1rem; }
    .data-val { font-family: monospace; font-weight: bold; color: var(--primary); }
    
    .input-group { display: flex; align-items: center; justify-content: space-between; margin-bottom: 12px; }
    .input-group label { width: 40px; color: var(--text-dim); font-weight: bold; }
    .input-group input { 
      background: rgba(0,0,0,0.3); border: 1px solid rgba(255,255,255,0.2); 
      color: white; padding: 10px; border-radius: 8px; width: calc(100% - 50px);
      font-family: monospace; font-size: 1rem;
    }
    .input-group input:focus { outline: none; border-color: var(--primary); box-shadow: 0 0 0 2px rgba(6,182,212,0.3); }
    
    .btn {
      width: 100%; background: linear-gradient(135deg, var(--primary), #3b82f6);
      border: none; color: white; padding: 14px; border-radius: 10px;
      font-size: 1.1rem; font-weight: bold; cursor: pointer;
      box-shadow: 0 4px 15px rgba(6, 182, 212, 0.4);
      transition: all 0.2s; margin-top: 10px;
    }
    .btn:hover { transform: scale(1.02); box-shadow: 0 6px 20px rgba(6, 182, 212, 0.6); }
    .btn:active { transform: scale(0.98); }
    
    .btn-secondary { background: rgba(255,255,255,0.1); box-shadow: none; border: 1px solid rgba(255,255,255,0.2); }
    .btn-secondary:hover { background: rgba(255,255,255,0.15); box-shadow: 0 4px 10px rgba(0,0,0,0.2); }
    .btn-danger { background: linear-gradient(135deg, #f43f5e, #e11d48); box-shadow: 0 4px 15px rgba(244, 63, 94, 0.4); }
    
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    
    .indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; background: #f43f5e; box-shadow: 0 0 8px #f43f5e;}
    .indicator.on { background: #10b981; box-shadow: 0 0 8px #10b981; }
    .pin-table { width:100%; border-collapse:collapse; font-size:0.8rem; }
    .pin-table th,.pin-table td { padding:5px 8px; border:1px solid rgba(255,255,255,0.08); }
    .pin-table th { background:rgba(255,255,255,0.05); color:var(--text-dim); font-size:0.7rem; text-transform:uppercase; }
    .cat-m { border-left:3px solid #3b82f6; }
    .cat-s { border-left:3px solid #10b981; }
    .cat-r { border-left:3px solid #f59e0b; }
    .cat-i { border-left:3px solid #a855f7; }
    .relay-dot { display:inline-block; width:10px; height:10px; border-radius:50%; background:#475569; margin-right:6px; transition:all 0.2s; }
    .relay-dot.active { background:#10b981; box-shadow:0 0 8px #10b981; }
    .jog-group { display:flex; gap:6px; align-items:center; margin-bottom:8px; }
    .jog-group label { width:50px; font-size:0.9rem; color:var(--text-dim); }
    .jog-btn { padding:7px 12px; border-radius:8px; border:1px solid rgba(255,255,255,0.2); background:rgba(255,255,255,0.08); color:white; cursor:pointer; font-size:0.85rem; transition:all 0.15s; }
    .jog-btn:hover { background:rgba(255,255,255,0.2); transform:scale(1.05); }
    .jog-btn:active { transform:scale(0.95); }
    .jog-input { width:60px; background:rgba(0,0,0,0.3); border:1px solid rgba(255,255,255,0.2); color:white; padding:5px; border-radius:6px; text-align:center; font-family:monospace; }
    .mpu-grid { display:grid; grid-template-columns:1fr 1fr 1fr; gap:4px; }
    .full-width { grid-column: 1 / -1; }
  </style>
</head>
<body>
  <h1><span>ZVD</span> 로봇팔 대시보드</h1>
  
  <div class="container">
    <!-- 좌측 패널: 상태 모니터링 -->
    <div class="glass-card">
      <h2>실시간 상태</h2>
      <div id="statusBadge" class="status-badge">연결 중...</div>
      
      <div class="data-row">
        <span>현재 위치 (X, Y, Z)</span>
        <span id="curPos" class="data-val">0.0, 0.0, 0.0</span>
      </div>
      <div class="data-row">
        <span>가속도 (ax, ay)</span>
        <span id="accelData" class="data-val">0.00, 0.00</span>
      </div>
      <div class="data-row">
        <span>고유 진동수 (fn)</span>
        <span id="freqData" class="data-val">8.00 Hz</span>
      </div>
      <div class="data-row">
        <span>감쇠비 (ζ)</span>
        <span id="zetaData" class="data-val">0.050</span>
      </div>
      
      <div style="margin-top: 30px;">
        <div class="grid-2">
          <button class="btn btn-secondary" onclick="sendCommand('rehome')">⌂ 강제 호밍</button>
          <button class="btn btn-secondary" onclick="sendCommand('tune')">∿ ZVD 튜닝</button>
        </div>
        <button class="btn" style="background:#f43f5e;width:100%;margin-top:8px;font-size:1.1rem;" onclick="sendCommand('estop')">⛔ 비상정지 (E-STOP)</button>
      </div>
    </div>
    
    <!-- 우측 패널: 티칭 설정 -->
    <div class="glass-card">
      <h2>📍 4점 티칭</h2>
      <p style="color:var(--text-dim);font-size:0.85rem;margin:0 0 15px;">조그로 팔을 이동 후 각 위치를 저장하세요.</p>
      
      <div style="margin-bottom:15px;">
        <h3 style="color:var(--primary); margin:0 0 8px; font-size:1rem;">Point A (피킹)</h3>
        <div id="ptAU" class="data-row"><span>A 위 (대기)</span><span class="data-val" style="color:#f43f5e;">❌</span></div>
        <button class="btn btn-secondary" style="padding:6px;width:100%;margin-bottom:4px;" onclick="sendCommand('saveAU')">📌 A위 저장 (대기높이)</button>
        <div id="ptAD" class="data-row"><span>A 아래 (집기)</span><span class="data-val" style="color:#f43f5e;">❌</span></div>
        <button class="btn btn-secondary" style="padding:6px;width:100%;" onclick="sendCommand('saveAD')">📌 A아래 저장 (칩높이)</button>
      </div>
      
      <div style="margin-bottom:15px;">
        <h3 style="color:var(--accent); margin:0 0 8px; font-size:1rem;">Point B (배치)</h3>
        <div id="ptBU" class="data-row"><span>B 위 (대기)</span><span class="data-val" style="color:#f43f5e;">❌</span></div>
        <button class="btn btn-secondary" style="padding:6px;width:100%;margin-bottom:4px;" onclick="sendCommand('saveBU')">📌 B위 저장 (대기높이)</button>
        <div id="ptBD" class="data-row"><span>B 아래 (놓기)</span><span class="data-val" style="color:#f43f5e;">❌</span></div>
        <button class="btn btn-secondary" style="padding:6px;width:100%;" onclick="sendCommand('saveBD')">📌 B아래 저장 (놓는높이)</button>
      </div>
      
      <div id="cycleInfo" class="data-row" style="margin-bottom:10px;"><span>사이클 횟수</span><span class="data-val">0</span></div>
      <button class="btn btn-danger" style="width:100%;" onclick="sendCommand('startCycle')">🔄 무한 반복 사이클 시작</button>
    </div>
  </div>

  <!-- 핀 배치도 -->
  <div class="glass-card" style="max-width:1000px; margin:20px auto;">
    <h2>📌 핀 배치도 (ESP32-S3)</h2>
    <table class="pin-table">
      <tr><th>기능</th><th>GPIO</th><th>연결 대상</th><th>비고</th></tr>
      <tr class="cat-m"><td>Base STEP</td><td>4</td><td>TB6600 #1 PUL+</td><td rowspan="3">베이스 축</td></tr>
      <tr class="cat-m"><td>Base DIR</td><td>5</td><td>TB6600 #1 DIR+</td></tr>
      <tr class="cat-m"><td>Base ENA</td><td>8</td><td>TB6600 #1 ENA+</td></tr>
      <tr class=\"cat-m\"><td>Shoulder STEP</td><td>15</td><td>TB6600 #2 PUL+</td><td rowspan=\"3\">숄더 축 (몸체쪽)</td></tr>
      <tr class=\"cat-m\"><td>Shoulder DIR</td><td>16</td><td>TB6600 #2 DIR+</td></tr>
      <tr class=\"cat-m\"><td>Shoulder ENA</td><td>10</td><td>TB6600 #2 ENA+</td></tr>
      <tr class=\"cat-m\"><td>Elbow STEP</td><td>6</td><td>TB6600 #3 PUL+</td><td rowspan=\"3\">엘보 축 (끝단쪽)</td></tr>
      <tr class=\"cat-m\"><td>Elbow DIR</td><td>7</td><td>TB6600 #3 DIR+</td></tr>
      <tr class=\"cat-m\"><td>Elbow ENA</td><td>9</td><td>TB6600 #3 ENA+</td></tr>
      <tr class=\"cat-s\"><td>Base Limit</td><td>1</td><td>리밋 스위치</td><td>NC (HIGH=눌림)</td></tr>
      <tr class=\"cat-s\"><td>Shoulder Limit</td><td>47</td><td>리밋 스위치</td><td>NO (LOW=눌림)</td></tr>
      <tr class=\"cat-s\"><td>Elbow Limit</td><td>2</td><td>리밋 스위치</td><td>NC (HIGH=눌림)</td></tr>
      <tr class="cat-r"><td>Vacuum Pump</td><td>21</td><td>릴레이 #1 IN</td><td>LOW=가동 (Open-Drain)</td></tr>
      <tr class="cat-i"><td>MPU6050 SDA</td><td>17</td><td>MPU6050</td><td rowspan="2">I2C 3.3V</td></tr>
      <tr class="cat-i"><td>MPU6050 SCL</td><td>18</td><td>MPU6050</td></tr>
    </table>
  </div>

  <!-- 하드웨어 테스트 패널 -->
  <div class="glass-card" style="max-width:1000px; margin:20px auto;">
    <h2>🔧 하드웨어 테스트 모드</h2>
    <div class="grid-2">
      <div>
        <h3 style="color:var(--text-dim); margin-bottom:10px;">리밋 스위치</h3>
        <div class="data-row"><span style="display:flex;align-items:center;"><span id="sw_b_ind" class="indicator"></span>베이스 (1)</span><span id="sw_b_txt">-</span></div>
        <div class="data-row"><span style="display:flex;align-items:center;"><span id="sw_s_ind" class="indicator"></span>숄더 (2)</span><span id="sw_s_txt">-</span></div>
        <div class="data-row"><span style="display:flex;align-items:center;"><span id="sw_e_ind" class="indicator"></span>엘보 (47)</span><span id="sw_e_txt">-</span></div>
        <h3 style="color:var(--text-dim); margin:15px 0 8px;">MPU6050 <span id="mpu_st" style="font-size:0.75rem;color:#f43f5e;">DISCONNECTED</span></h3>
        <div class="mpu-grid">
          <div class="data-row" style="font-size:0.85rem;"><span>ax</span><span id="m_ax" class="data-val">0.00</span></div>
          <div class="data-row" style="font-size:0.85rem;"><span>ay</span><span id="m_ay" class="data-val">0.00</span></div>
          <div class="data-row" style="font-size:0.85rem;"><span>az</span><span id="m_az" class="data-val">0.00</span></div>
          <div class="data-row" style="font-size:0.85rem;"><span>gx</span><span id="m_gx" class="data-val">0.00</span></div>
          <div class="data-row" style="font-size:0.85rem;"><span>gy</span><span id="m_gy" class="data-val">0.00</span></div>
          <div class="data-row" style="font-size:0.85rem;"><span>gz</span><span id="m_gz" class="data-val">0.00</span></div>
        </div>
      </div>
      <div>
        <h3 style="color:var(--text-dim); margin-bottom:10px;">릴레이 제어</h3>
        <div class="data-row" style="margin-bottom:15px;"><span style="display:flex;align-items:center;"><span id="rp_dot" class="relay-dot"></span>진공 펌프</span>
          <span><button class="jog-btn" onclick="testCmd('pump',1)">ON</button> <button class="jog-btn" onclick="testCmd('pump',0)">OFF</button></span></div>
        <h3 style="color:var(--text-dim); margin:15px 0 8px;">관절 모터 조그</h3>
        <div style="margin-bottom:8px;"><label style="color:var(--text-dim);font-size:0.85rem;">스텝 수: </label><input type="number" id="jogN" class="jog-input" value="200" min="1" max="5000"></div>
        <div class="jog-group"><label>베이스</label><button class="jog-btn" onclick="jog('base',-1)">◀ CCW</button><button class="jog-btn" onclick="jog('base',1)">CW ▶</button></div>
        <div class="jog-group"><label>숄더</label><button class="jog-btn" onclick="jog('shoulder',-1)">◀ CCW</button><button class="jog-btn" onclick="jog('shoulder',1)">CW ▶</button></div>
        <div class="jog-group"><label>엘보</label><button class="jog-btn" onclick="jog('elbow',-1)">◀ CCW</button><button class="jog-btn" onclick="jog('elbow',1)">CW ▶</button></div>
        
        <h3 style="color:#10b981; margin:15px 0 8px;">↕ 수직 조그 (자코비안)</h3>
        <div style="margin-bottom:8px;"><label style="color:var(--text-dim);font-size:0.85rem;">mm: </label><input type="number" id="jogZmm" class="jog-input" value="5" min="1" max="50"></div>
        <div class="jog-group"><label>Z축</label><button class="jog-btn" style="background:#10b981;" onclick="jogZ(1)">▲ Z+(상)</button><button class="jog-btn" style="background:#f43f5e;" onclick="jogZ(-1)">▼ Z-(하)</button></div>
        
        <h3 style="color:var(--text-dim); margin:15px 0 8px;">직교 좌표 조그 (IK)</h3>
        <div style="margin-bottom:8px;"><label style="color:var(--text-dim);font-size:0.85rem;">거리(mm): </label><input type="number" id="jogXyzDist" class="jog-input" value="10" min="1" max="100"></div>
        <div class="jog-group"><label>X 축</label><button class="jog-btn" onclick="jogXYZ('x',-1)">-X (좌)</button><button class="jog-btn" onclick="jogXYZ('x',1)">+X (우)</button></div>
        <div class="jog-group"><label>Y 축</label><button class="jog-btn" onclick="jogXYZ('y',-1)">-Y (후)</button><button class="jog-btn" onclick="jogXYZ('y',1)">+Y (전)</button></div>
        <div class="jog-group"><label>Z 축</label><button class="jog-btn" onclick="jogXYZ('z',-1)">-Z (하)</button><button class="jog-btn" onclick="jogXYZ('z',1)">+Z (상)</button></div>
      </div>
    </div>
  </div>

  <script>
    setInterval(updateStatus, 200); // 1000ms -> 200ms 로 갱신 주기 단축
    updateStatus(true);

    function updateStatus(initInputs = false) {
      fetch('/api/status')
        .then(res => res.json())
        .then(data => {
          const badge = document.getElementById('statusBadge');
          badge.innerText = data.state_str;
          if(data.state_idx === 1) { // STATE_IDLE
            badge.className = "status-badge";
          } else {
            badge.className = "status-badge active";
          }
          
          document.getElementById('accelData').innerText = `${data.ax.toFixed(2)}, ${data.ay.toFixed(2)}`;
          document.getElementById('freqData').innerText = `${data.freq.toFixed(2)} Hz`;
          document.getElementById('zetaData').innerText = `${data.zeta.toFixed(3)}`;
          const elPos = document.getElementById('curPos');
          if(elPos && data.cur_pos) elPos.innerText = `${data.cur_pos.x.toFixed(1)}, ${data.cur_pos.y.toFixed(1)}, ${data.cur_pos.z.toFixed(1)}`;
          const setSw = (id, val) => {
            document.getElementById(id+'_ind').className = val === 1 ? 'indicator on' : 'indicator';
            document.getElementById(id+'_txt').innerText = val === 1 ? 'ON' : 'OFF';
            document.getElementById(id+'_txt').style.color = val === 1 ? '#10b981' : 'var(--primary)';
          };
          setSw('sw_b', data.sw_base); setSw('sw_s', data.sw_shoulder); setSw('sw_e', data.sw_elbow);
          ['ax','ay','az','gx','gy','gz'].forEach(k => { const el=document.getElementById('m_'+k); if(el && data[k]!=null) el.innerText=data[k].toFixed(2); });
          const ms=document.getElementById('mpu_st'); if(ms){ms.innerText=data.mpu_ok?'CONNECTED':'DISCONNECTED'; ms.style.color=data.mpu_ok?'#10b981':'#f43f5e';}
          document.getElementById('rp_dot').className = data.relay_pump ? 'relay-dot active' : 'relay-dot';
          // 4점 티칭 상태 표시
          ['AU','AD','BU','BD'].forEach(k => {
            const el = document.getElementById('pt'+k);
            if(el) { const ok = data['pt'+k]; el.querySelector('.data-val').innerHTML = ok ? '✅' : '❌'; el.querySelector('.data-val').style.color = ok ? '#10b981' : '#f43f5e'; }
          });
          const cycInfo = document.getElementById('cycleInfo');
          if(cycInfo) cycInfo.innerHTML = '<span>사이클 횟수</span><span class="data-val">'+data.cycles+'</span>';
        }).catch(err => console.error(err));
    }
    function sendCommand(cmd) {
      fetch('/api/command?c=' + cmd, { method: 'POST' }).then(res => { if(res.ok) updateStatus(); });
    }

    function testCmd(target, state) {
      const f = new URLSearchParams(); f.append('target',target); f.append('state',state);
      fetch('/api/test_cmd', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:f }).then(r => { if(!r.ok) alert('BUSY'); });
    }
    function jog(axis, dir) {
      const n = document.getElementById('jogN').value || 50;
      const f = new URLSearchParams(); f.append('target','jog_'+axis); f.append('state',dir); f.append('steps',n);
      fetch('/api/test_cmd', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:f }).then(r => { if(!r.ok) alert('BUSY'); });
    }
    function jogZ(dir) {
      const mm = document.getElementById('jogZmm').value || 5;
      const f = new URLSearchParams(); f.append('target','jog_z'); f.append('state',dir); f.append('steps',mm);
      fetch('/api/test_cmd', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:f }).then(r => { if(!r.ok) alert('RANGE'); });
    }
    function jogXYZ(axis, dir) {
      const d = document.getElementById('jogXyzDist').value || 10;
      const f = new URLSearchParams(); f.append('axis', axis); f.append('dist', d * dir);
      fetch('/api/jog_xyz', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:f }).then(r => { if(!r.ok) alert('BUSY/IK Fail'); });
    }
  </script>
</body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 7: 시스템 함수 (MPU, IK, ZVD, 모터, 호밍)
// ═══════════════════════════════════════════════════════════════════════════

bool hardware_bypassed = false; // 항상 실제 하드웨어 구동 모드로 설정
bool mpu_connected = false;
bool isHomed = false; // 호밍 완료 여부 추적

// 모터 활성화/비활성화 (TB6600 Common Anode: LOW=전류흐름(비활성), HIGH=오픈드레인 차단(활성))
void motorsEnable() {
  digitalWrite(MOTOR1_BASE_ENA, HIGH);  // 베이스도 항상 활성화 (위치 유지)
  digitalWrite(MOTOR2_SHOULDER_ENA, HIGH);
  digitalWrite(MOTOR3_ELBOW_ENA, HIGH);
  Serial.println("[MOTOR] All motors enabled");
}
void motorsDisable() {
  digitalWrite(MOTOR1_BASE_ENA, LOW);
  digitalWrite(MOTOR2_SHOULDER_ENA, LOW);
  digitalWrite(MOTOR3_ELBOW_ENA, LOW);
  Serial.println("[MOTOR] Disabled (free spin)");
}

void mpu6050_init() {
  Wire.begin(MPU6050_SDA, MPU6050_SCL, 400000);
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    mpu_connected = true;
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true);
    Serial.println("[MPU6050] 초기화 완료");
  } else {
    Serial.println("[MPU6050] 센서 미연결 - 가상 시뮬레이션 모드로 작동합니다.");
  }
}

MPU6050_Data mpu6050_read() {
  MPU6050_Data data = {0};
  if (!mpu_connected) return data; // 미연결 시 기본값 0 리턴
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() >= 14) {
    data.ax = ((int16_t)((Wire.read() << 8) | Wire.read())) / 16384.0f;
    data.ay = ((int16_t)((Wire.read() << 8) | Wire.read())) / 16384.0f;
    data.az = ((int16_t)((Wire.read() << 8) | Wire.read())) / 16384.0f;
    int16_t temp = (Wire.read() << 8) | Wire.read(); (void)temp;
    data.gx = ((int16_t)((Wire.read() << 8) | Wire.read())) / 131.0f;
    data.gy = ((int16_t)((Wire.read() << 8) | Wire.read())) / 131.0f;
    data.gz = ((int16_t)((Wire.read() << 8) | Wire.read())) / 131.0f;
  }
  return data;
}

void estimateVibrationParams() {
  Serial.println("[ZVD 튜닝] 고유 진동수 측정 (2초간)...");
  if (!mpu_connected) {
    Serial.println("→ 센서 없음: 기본값(fn=8Hz, zeta=0.05) 유지");
    return;
  }
  const int SC = 400; const int SI = 5; float samples[SC];
  for (int i = 0; i < SC; i++) {
    MPU6050_Data d = mpu6050_read();
    samples[i] = sqrtf(d.ax * d.ax + d.ay * d.ay);
    delay(SI);
  }
  float pv[20]; int pt[20]; int pc = 0;
  float avg = 0; for (int j = 0; j < SC; j++) avg += samples[j]; avg /= SC;
  for (int i = 1; i < SC - 1 && pc < 20; i++) {
    if (samples[i] > samples[i - 1] && samples[i] > samples[i + 1]) {
      if (samples[i] > avg * 1.2f) { pv[pc] = samples[i]; pt[pc] = i; pc++; }
    }
  }
  if (pc >= 3) {
    float totalPeriod = 0;
    for (int i = 1; i < pc; i++) totalPeriod += (pt[i] - pt[i - 1]) * SI;
    float avgP = totalPeriod / (pc - 1);
    float freq = 1000.0f / avgP;
    float logDec = 0; int logCount = 0;
    for (int i = 0; i < pc - 1; i++) {
      if (pv[i + 1] > 0.001f && pv[i] > pv[i + 1]) { logDec += logf(pv[i] / pv[i + 1]); logCount++; }
    }
    if (logCount > 0 && freq > 1.0f && freq < 50.0f) {
      float delta = logDec / logCount;
      float zeta = delta / sqrtf(4.0f * PI * PI + delta * delta);
      zvd_natural_freq_hz = freq; zvd_damping_ratio = constrain(zeta, 0.001f, 0.5f);
      Serial.printf("→ 고유진동수: %.2fHz, 감쇠비: %.4f\n", zvd_natural_freq_hz, zvd_damping_ratio);
    } else Serial.println("→ 측정 실패 (기본값 유지)");
  } else Serial.println("→ 진동 감지 불가 (기본값 유지)");
}

JointAngles solveIK(float x, float y, float z) {
  JointAngles res; res.valid = false;
  res.base_deg = atan2f(x, y) * (180.0f / PI);
  float r = sqrtf(x * x + y * y) - END_EFFECTOR_OFFSET;
  float z_prime = z - BASE_HEIGHT;
  float dist_sq = r * r + z_prime * z_prime;
  float max_r = LOW_SHANK_LENGTH + HIGH_SHANK_LENGTH;
  float min_r = fabsf(LOW_SHANK_LENGTH - HIGH_SHANK_LENGTH);
  float dist = sqrtf(dist_sq);
  if (dist > max_r || dist < min_r || r < 0) return res;

  float cos_el = (dist_sq - LOW_SHANK_LENGTH * LOW_SHANK_LENGTH - HIGH_SHANK_LENGTH * HIGH_SHANK_LENGTH) / (2.0f * LOW_SHANK_LENGTH * HIGH_SHANK_LENGTH);
  cos_el = constrain(cos_el, -1.0f, 1.0f);
  float th_el = acosf(cos_el);
  float k1 = LOW_SHANK_LENGTH + HIGH_SHANK_LENGTH * cos_el;
  float k2 = HIGH_SHANK_LENGTH * sinf(th_el);
  float th_sh = atan2f(z_prime, r) - atan2f(k2, k1);

  res.shoulder_deg = th_sh * (180.0f / PI);
  res.elbow_deg = th_el * (180.0f / PI);
  res.valid = true;
  return res;
}

// 순운동학: 현재 스텝 → XYZ (관절 조그 후에도 정확한 좌표)
CartesianPoint solveFK() {
  int32_t sb, ss, se;
  portENTER_CRITICAL(&stepsMux);
  sb = currentSteps_base;
  ss = currentSteps_shoulder;
  se = currentSteps_elbow;
  portEXIT_CRITICAL(&stepsMux);
  float base_rad = (sb / STEPS_PER_DEG_BASE) * PI / 180.0f;
  float sh_rad   = (ss / STEPS_PER_DEG_SHOULDER) * PI / 180.0f;
  float el_rad   = (se / STEPS_PER_DEG_ELBOW) * PI / 180.0f;
  float r = LOW_SHANK_LENGTH * cosf(sh_rad)
          + HIGH_SHANK_LENGTH * cosf(sh_rad + el_rad)
          + END_EFFECTOR_OFFSET;
  float z = LOW_SHANK_LENGTH * sinf(sh_rad)
          + HIGH_SHANK_LENGTH * sinf(sh_rad + el_rad)
          + BASE_HEIGHT;
  CartesianPoint p;
  p.x = r * sinf(base_rad);
  p.y = r * cosf(base_rad);
  p.z = z;
  return p;
}

void anglesToSteps(const JointAngles& angles, int32_t& b, int32_t& s, int32_t& e) {
  b = (int32_t)(angles.base_deg * STEPS_PER_DEG_BASE);
  s = (int32_t)(angles.shoulder_deg * STEPS_PER_DEG_SHOULDER);
  e = (int32_t)(angles.elbow_deg * STEPS_PER_DEG_ELBOW);
}

ZVD_Impulse calculateZVD(float freq_hz, float zeta) {
  ZVD_Impulse zvd;
  float omega_d = 2.0f * PI * freq_hz * sqrtf(1.0f - zeta * zeta);
  float K = expf(-zeta * PI / sqrtf(1.0f - zeta * zeta));
  float denom = (1.0f + K) * (1.0f + K);
  zvd.amplitude[0] = 1.0f / denom; zvd.amplitude[1] = 2.0f * K / denom; zvd.amplitude[2] = K * K / denom;
  float dt = (PI / omega_d) * 1000.0f;
  zvd.delay_ms[0] = 0.0f; zvd.delay_ms[1] = dt; zvd.delay_ms[2] = 2.0f * dt;
  return zvd;
}

inline void stepPulse(int pin) {
  // Common Anode (PUL- 연결) 방식: LOW일 때 전류가 흐르고, HIGH(Open-Drain)일 때 차단됨.
  digitalWrite(pin, LOW);
  delayMicroseconds(100); 
  digitalWrite(pin, HIGH);
}

uint32_t trapezoidalProfile(int32_t cs, int32_t ts, uint32_t baseIval) {
  if (ts <= 0) return baseIval;
  int32_t as = min((int32_t)ACCEL_STEPS, ts / 3);
  int32_t ds = ts - as;
  float f = (cs < as) ? (float)cs / as : ((cs >= ds) ? (float)(ts - cs) / as : 1.0f);
  f = constrain(f, 0.05f, 1.0f);
  return constrain((uint32_t)(MAX_STEP_INTERVAL_US - (MAX_STEP_INTERVAL_US - baseIval) * f), MIN_STEP_INTERVAL_US, MAX_STEP_INTERVAL_US);
}

void executeMotion(int32_t tb, int32_t ts, int32_t te, uint32_t base_ival) {
  int32_t db = tb - currentSteps_base, ds = ts - currentSteps_shoulder, de = te - currentSteps_elbow;
  int8_t dirb = (db >= 0) ? 1 : -1, dirs = (ds >= 0) ? 1 : -1, dire = (de >= 0) ? 1 : -1;
  // Common Anode로 인해 DIR 논리가 반전됨 (<= 0 이면 정방향).
  // GPIO 6/7 모터(엘보우)만 추가 반전 필요 (> 0).
  digitalWrite(MOTOR1_BASE_DIR, dirb <= 0); digitalWrite(MOTOR2_SHOULDER_DIR, dirs <= 0); digitalWrite(MOTOR3_ELBOW_DIR, dire > 0);
  delayMicroseconds(5);
  int32_t ab = abs(db), as = abs(ds), ae = abs(de);
  int32_t ms = max(ab, max(as, ae));
  if (ms == 0) { motionComplete = true; return; }
  int32_t eb = ms / 2, es = ms / 2, ee = ms / 2;
  motionComplete = false;
  for (int32_t st = 0; st < ms; st++) {
    uint32_t ival = trapezoidalProfile(st, ms, base_ival);
    eb -= ab; if (eb < 0) { eb += ms; stepPulse(MOTOR1_BASE_STEP); currentSteps_base += dirb; }
    es -= as; if (es < 0) { es += ms; stepPulse(MOTOR2_SHOULDER_STEP); currentSteps_shoulder += dirs; }
    ee -= ae; if (ee < 0) { ee += ms; stepPulse(MOTOR3_ELBOW_STEP); currentSteps_elbow += dire; }
    delayMicroseconds(ival);

    // 400스텝마다 1ms 휴식: Watchdog 방지 및 이동 중 웹 응답(E-STOP 등) 가능하게 함
    if (st > 0 && st % 400 == 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  currentSteps_base = tb; currentSteps_shoulder = ts; currentSteps_elbow = te;
  motionComplete = true;
}

void executeMotionZVD(int32_t tb, int32_t ts, int32_t te, uint32_t base_ival) {
  ZVD_Impulse zvd = calculateZVD(zvd_natural_freq_hz, zvd_damping_ratio);
  // 시작 위치를 고정 저장 (임펄스 간 기준점이 밀리지 않도록)
  int32_t start_b = currentSteps_base;
  int32_t start_s = currentSteps_shoulder;
  int32_t start_e = currentSteps_elbow;
  int32_t d_b = tb - start_b, d_s = ts - start_s, d_e = te - start_e;
  for (int i = 0; i < ZVD_NUM_IMPULSES; i++) {
    if (i > 0) {
      float w = zvd.delay_ms[i] - zvd.delay_ms[i - 1];
      if (w > 0) vTaskDelay(pdMS_TO_TICKS(w));
    }
    int32_t p_tb = tb, p_ts = ts, p_te = te;
    if (i != ZVD_NUM_IMPULSES - 1) {
      float cr = 0; for (int j = 0; j <= i; j++) cr += zvd.amplitude[j];
      p_tb = start_b + (int32_t)(d_b * cr + 0.5f);
      p_ts = start_s + (int32_t)(d_s * cr + 0.5f);
      p_te = start_e + (int32_t)(d_e * cr + 0.5f);
    }
    executeMotion(p_tb, p_ts, p_te, base_ival);
  }
}

bool homeSingleAxis(int pinStep, int pinDir, int pinLim, int activeLevel, int homingDir,
                    int homeOffset, float initialDeg, float stepsPerDeg,
                    volatile int32_t& ctr, const char* name) {
  if (hardware_bypassed) {
    ctr = (int32_t)(initialDeg * stepsPerDeg); delay(100);
    Serial.printf("[HOME] %s bypassed (ctr=%d)\n", name, (int)ctr);
    return true;
  }
  int backDir = homingDir == LOW ? HIGH : LOW;
  int32_t maxS = STEPS_PER_REV * MICROSTEPS * GEAR_RATIO_BASE * 2;

  // 1단계: 이미 스위치 위에 있으면 빠져나가기 (Backoff)
  if (digitalRead(pinLim) == activeLevel) {
    Serial.printf("[HOME] %s on switch, backing off...\n", name);
    digitalWrite(pinDir, backDir); delayMicroseconds(5);
    for (int i = 0; i < HOMING_BACKOFF_STEPS * 3; i++) {
      if (digitalRead(pinLim) != activeLevel) break;
      stepPulse(pinStep); delayMicroseconds(HOMING_SPEED_US);
      if (i > 0 && i % 50 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
    }
    if (digitalRead(pinLim) == activeLevel) {
      Serial.printf("[HOME] %s backoff FAIL\n", name); 
      if (pinStep == MOTOR1_BASE_STEP) digitalWrite(MOTOR1_BASE_ENA, HIGH);
      return false;
    }
    delay(HOME_DWELL_MS);
  }

  // 2단계: 스위치 방향으로 접근 (Approach)
  Serial.printf("[HOME] %s approaching switch...\n", name);
  digitalWrite(pinDir, homingDir); delayMicroseconds(5);
  bool found = false;
  for (int32_t i = 0; i < maxS; i++) {
    if (webCmd_EStop) { 
      Serial.println("[HOME] E-STOP!"); 
      if (pinStep == MOTOR1_BASE_STEP) digitalWrite(MOTOR1_BASE_ENA, HIGH);
      return false; 
    }
    if (digitalRead(pinLim) == activeLevel) { found = true; break; }
    stepPulse(pinStep); delayMicroseconds(HOMING_SPEED_US);
    if (i > 0 && i % 50 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
  }
  if (!found) { 
    Serial.printf("[HOME] %s approach FAIL\n", name); 
    if (pinStep == MOTOR1_BASE_STEP) digitalWrite(MOTOR1_BASE_ENA, HIGH);
    return false; 
  }

  // 3단계: 살짝 빠져나가서 정밀 재접근
  delay(HOME_DWELL_MS);
  digitalWrite(pinDir, backDir); delayMicroseconds(5);
  for (int i = 0; i < HOMING_BACKOFF_STEPS; i++) {
    stepPulse(pinStep); delayMicroseconds(HOMING_SPEED_US * 2);
    if (i > 0 && i % 20 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
  }
  delay(HOME_DWELL_MS);
  digitalWrite(pinDir, homingDir); delayMicroseconds(5);
  for (int32_t i = 0; i < HOMING_BACKOFF_STEPS * 3; i++) {
    if (digitalRead(pinLim) == activeLevel) break;
    stepPulse(pinStep); delayMicroseconds(HOMING_APPROACH_SPEED_US);
    if (i > 0 && i % 20 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
  }

  // 4단계: 스위치에서 homeOffset 스텝만큼 물리적으로 되돌아가서 초기 자세에 도달
  //        (원작 endstop.cpp homeOffset() 함수와 동일한 동작)
  delay(HOME_DWELL_MS);
  Serial.printf("[HOME] %s moving to initial pose (%d steps)...\n", name, homeOffset);
  digitalWrite(pinDir, backDir); delayMicroseconds(5);
  for (int i = 0; i < homeOffset; i++) {
    stepPulse(pinStep); delayMicroseconds(HOMING_SPEED_US);
    if (i > 0 && i % 50 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
  }

  // 5단계: 초기 자세 각도를 스텝 카운터에 설정
  ctr = (int32_t)(initialDeg * stepsPerDeg);
  delay(HOME_DWELL_MS);
  Serial.printf("[HOME] %s OK (initial=%.1f deg, ctr=%d)\n", name, initialDeg, (int)ctr);
  return true;
}

bool homeAllAxes() {
  // 호밍 순서: 숄더(몸체쪽) → 엘보우(끝단쪽) → 베이스 (원작 homeSequence와 동일)
  bool ok = homeSingleAxis(MOTOR2_SHOULDER_STEP, MOTOR2_SHOULDER_DIR, LIMIT_SHOULDER,
              LIMIT_SHOULDER_ACTIVE, HOMING_DIR_SHOULDER, HOME_OFFSET_SHOULDER,
              HOME_INITIAL_DEG_SHOULDER, STEPS_PER_DEG_SHOULDER,
              currentSteps_shoulder, "Shoulder") &&
            homeSingleAxis(MOTOR3_ELBOW_STEP, MOTOR3_ELBOW_DIR, LIMIT_ELBOW,
              LIMIT_ELBOW_ACTIVE, HOMING_DIR_ELBOW, HOME_OFFSET_ELBOW,
              HOME_INITIAL_DEG_ELBOW, STEPS_PER_DEG_ELBOW,
              currentSteps_elbow, "Elbow") &&
            homeSingleAxis(MOTOR1_BASE_STEP, MOTOR1_BASE_DIR, LIMIT_BASE,
              LIMIT_BASE_ACTIVE, HOMING_DIR_BASE, HOME_OFFSET_BASE,
              HOME_INITIAL_DEG_BASE, STEPS_PER_DEG_BASE,
              currentSteps_base, "Base");
  return ok;
}

bool moveToXYZ(float x, float y, float z, bool zvd, uint32_t spd) {
  // Z 하한 안전 클램프
  if (z < Z_MIN_SAFE) z = Z_MIN_SAFE;
  JointAngles a = solveIK(x, y, z);
  if (!a.valid) return false;
  int32_t tb, ts, te; anglesToSteps(a, tb, ts, te);
  MotionCommand cmd = {tb, ts, te, spd, zvd, false};
  if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
  if (xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000)) != pdTRUE) return false;
  return true;
}

// 카테시안 직선 보간 이동 (순수 수직/수평 보장)
bool moveLinearXYZ(float tx, float ty, float tz, uint32_t spd) {
  if (tz < Z_MIN_SAFE) tz = Z_MIN_SAFE;
  CartesianPoint cur = solveFK();
  float dx = tx - cur.x, dy = ty - cur.y, dz = tz - cur.z;
  float dist = sqrtf(dx*dx + dy*dy + dz*dz);
  if (dist < 0.5f) return true; // 이미 도착
  int segments = (int)(dist / CARTESIAN_INTERP_MM);
  if (segments < 1) segments = 1;
  for (int i = 1; i <= segments; i++) {
    float t = (float)i / segments;
    float ix = cur.x + dx * t;
    float iy = cur.y + dy * t;
    float iz = cur.z + dz * t;
    if (!moveToXYZ(ix, iy, iz, false, spd)) return false;
  }
  return true;
}

void vacuumGrip(uint32_t w) { digitalWrite(RELAY_VACUUM_PUMP, RELAY_ON); delay(w); }
void vacuumRelease(uint32_t w) { digitalWrite(RELAY_VACUUM_PUMP, RELAY_OFF); delay(w); }

void taskStepper(void* pv) {
  MotionCommand cmd;
  while (true) {
    if (xQueueReceive(motionQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      if (cmd.use_zvd) executeMotionZVD(cmd.target_steps_base, cmd.target_steps_shoulder, cmd.target_steps_elbow, cmd.step_interval_us);
      else executeMotion(cmd.target_steps_base, cmd.target_steps_shoulder, cmd.target_steps_elbow, cmd.step_interval_us);
      xSemaphoreGive(motionDoneSem);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 8: Core 0 메인 제어 태스크 (상태 머신 + 웹 GUI 트리거 처리)
// ═══════════════════════════════════════════════════════════════════════════

void taskControl(void* pv) {
  while (true) {
    // 우선적인 웹 명령 처리
    if (webCmd_EStop) {
      webCmd_EStop = false;
      webCmd_ContinuousCycle = false;  // 사이클 중단
      digitalWrite(RELAY_VACUUM_PUMP, RELAY_OFF);
      motorsDisable();  // 모터 비활성화 (홀딩토크 OFF, 자유 회전)
      currentState = STATE_IDLE;
      Serial.println("[STATE] E-STOP -> IDLE (motors disabled)");
    }
    if (webCmd_Rehome) {
      webCmd_Rehome = false;
      motorsEnable();  // 호밍 전 모터 재활성화
      currentState = STATE_HOMING;
    }
    if (webCmd_TuneZVD) { webCmd_TuneZVD = false; estimateVibrationParams(); }

    switch (currentState) {
      case STATE_HOMING:
        if (homeAllAxes()) {
          estimateVibrationParams();
          isHomed = true;
          currentState = STATE_IDLE;
          Serial.println("[STATE] IDLE");
        } else { vTaskDelay(pdMS_TO_TICKS(5000)); }
        break;

      case STATE_IDLE: {
        // 직교 조그 처리
        if (webCmd_JogXYZ_Axis != 0) {
          if (!isHomed) {
            Serial.println("[ERROR] Must HOME before using Cartesian XYZ Jog!");
            webCmd_JogXYZ_Axis = 0;
          } else {
            CartesianPoint cur = solveFK();
            float tx = cur.x, ty = cur.y, tz = cur.z;
            if (webCmd_JogXYZ_Axis == 'x') tx += webCmd_JogXYZ_Dist;
            if (webCmd_JogXYZ_Axis == 'y') ty += webCmd_JogXYZ_Dist;
            if (webCmd_JogXYZ_Axis == 'z') tz += webCmd_JogXYZ_Dist;
            webCmd_JogXYZ_Axis = 0;
            if (!moveLinearXYZ(tx, ty, tz, 1500)) {
              Serial.println("IK fail (out of range)");
            }
          }
        }
        // 4점 위치 저장 처리
        auto doSave = [](volatile bool& flag, TeachPoint& pt, const char* name) {
          if (!flag) return;
          flag = false;
          portENTER_CRITICAL(&stepsMux);
          pt.steps_base = currentSteps_base;
          pt.steps_shoulder = currentSteps_shoulder;
          pt.steps_elbow = currentSteps_elbow;
          portEXIT_CRITICAL(&stepsMux);
          pt.saved = true;
          Serial.printf("[TEACH] %s saved: steps(%d,%d,%d)\n", name, (int)pt.steps_base, (int)pt.steps_shoulder, (int)pt.steps_elbow);
          saveTeachPoints();
        };
        doSave(webCmd_SaveAU, pointA_up, "A위");
        doSave(webCmd_SaveAD, pointA_down, "A아래");
        doSave(webCmd_SaveBU, pointB_up, "B위");
        doSave(webCmd_SaveBD, pointB_down, "B아래");
        // 사이클 시작
        if (webCmd_ContinuousCycle) {
          if (!isHomed || !pointA_up.saved || !pointA_down.saved || !pointB_up.saved || !pointB_down.saved) {
            Serial.println("[ERROR] HOME + 4점 모두 저장 필요!");
            webCmd_ContinuousCycle = false;
          } else {
            digitalWrite(RELAY_VACUUM_PUMP, RELAY_OFF);
            cycleCount = 0;
            currentState = STATE_MOVE_TO_A;
            Serial.println("[CYCLE] 무한 반복 사이클 시작!");
          }
        }
        break;
      }

      // ── 사이클: A위로 이동 ──
      case STATE_MOVE_TO_A: {
        Serial.printf("[CYCLE] → A위 (steps: %d,%d,%d)\n",
          (int)pointA_up.steps_base, (int)pointA_up.steps_shoulder, (int)pointA_up.steps_elbow);
        MotionCommand cmd = {pointA_up.steps_base, pointA_up.steps_shoulder, pointA_up.steps_elbow, 800, true, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(200);
        currentState = STATE_DESCEND_A;
        break;
      }

      // ── A아래로 하강 (관절 직접 이동, IK 없음) ──
      case STATE_DESCEND_A: {
        Serial.println("[CYCLE] A↓");
        MotionCommand cmd = {pointA_down.steps_base, pointA_down.steps_shoulder, pointA_down.steps_elbow, 1200, false, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(100);
        currentState = STATE_PICK;
        break;
      }

      // ── 진공 흡착 ──
      case STATE_PICK:
        vacuumGrip(500);
        currentState = STATE_ASCEND_A;
        break;

      // ── A위로 상승 ──
      case STATE_ASCEND_A: {
        MotionCommand cmd = {pointA_up.steps_base, pointA_up.steps_shoulder, pointA_up.steps_elbow, 800, true, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(200);
        currentState = STATE_MOVE_TO_B;
        break;
      }

      // ── B위로 이동 ──
      case STATE_MOVE_TO_B: {
        Serial.printf("[CYCLE] → B위 (steps: %d,%d,%d)\n",
          (int)pointB_up.steps_base, (int)pointB_up.steps_shoulder, (int)pointB_up.steps_elbow);
        MotionCommand cmd = {pointB_up.steps_base, pointB_up.steps_shoulder, pointB_up.steps_elbow, 800, true, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(200);
        currentState = STATE_DESCEND_B;
        break;
      }

      // ── B아래로 하강 ──
      case STATE_DESCEND_B: {
        Serial.println("[CYCLE] B↓");
        MotionCommand cmd = {pointB_down.steps_base, pointB_down.steps_shoulder, pointB_down.steps_elbow, 1200, false, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(100);
        currentState = STATE_PLACE;
        break;
      }

      // ── 진공 해제 ──
      case STATE_PLACE:
        vacuumRelease(200);
        currentState = STATE_ASCEND_B;
        break;

      // ── B위로 상승 후 반복 ──
      case STATE_ASCEND_B: {
        MotionCommand cmd = {pointB_up.steps_base, pointB_up.steps_shoulder, pointB_up.steps_elbow, 800, true, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(200);
        currentState = STATE_DESCEND_B2;  // → Phase 2: B에서 다시 집기
        break;
      }

      // ═══ Phase 2: B→A (되가져오기) ═══

      // ── B아래로 하강 (다시 집기) ──
      case STATE_DESCEND_B2: {
        Serial.println("[CYCLE] B↓(집기)");
        MotionCommand cmd = {pointB_down.steps_base, pointB_down.steps_shoulder, pointB_down.steps_elbow, 1200, false, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(100);
        currentState = STATE_PICK_B;
        break;
      }

      case STATE_PICK_B:
        vacuumGrip(500);
        currentState = STATE_ASCEND_B2;
        break;

      case STATE_ASCEND_B2: {
        MotionCommand cmd = {pointB_up.steps_base, pointB_up.steps_shoulder, pointB_up.steps_elbow, 800, true, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(200);
        currentState = STATE_RETURN_A;
        break;
      }

      // ── A위로 복귀 ──
      case STATE_RETURN_A: {
        Serial.println("[CYCLE] →A위(복귀)");
        MotionCommand cmd = {pointA_up.steps_base, pointA_up.steps_shoulder, pointA_up.steps_elbow, 800, true, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(200);
        currentState = STATE_DESCEND_A2;
        break;
      }

      // ── A아래로 하강 (놓기) ──
      case STATE_DESCEND_A2: {
        Serial.println("[CYCLE] A↓(놓기)");
        MotionCommand cmd = {pointA_down.steps_base, pointA_down.steps_shoulder, pointA_down.steps_elbow, 1200, false, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(100);
        currentState = STATE_PLACE_A;
        break;
      }

      case STATE_PLACE_A:
        vacuumRelease(200);
        currentState = STATE_ASCEND_A2;
        break;

      // ── A위로 상승 → 1사이클 완료 ──
      case STATE_ASCEND_A2: {
        MotionCommand cmd = {pointA_up.steps_base, pointA_up.steps_shoulder, pointA_up.steps_elbow, 800, true, false};
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
          xSemaphoreTake(motionDoneSem, pdMS_TO_TICKS(30000));
        }
        delay(200);
        cycleCount++;
        Serial.printf("[CYCLE] 사이클 #%d 완료! (왕복)\n", (int)cycleCount);
        if (webCmd_ContinuousCycle) {
          currentState = STATE_MOVE_TO_A;  // 다시 Phase 1
        } else {
          currentState = STATE_IDLE;
        }
        break;
      }
        break;
    }

    // 센서는 loop()에서 읽음 (웹서버와 같은 주기로 항상 갱신)
    
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 9: 웹 서버 초기화 함수
// ═══════════════════════════════════════════════════════════════════════════

void setupWiFiAndWeb() {
  Serial.println("[WIFI] AP 모드 초기화 시작...");
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("RobotArm_ZVD_WiFi", "12345678", 1, 0, 4);
  WiFi.setSleep(false); // 와이파이 절전모드 해제 (연결 안정성 향상)
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("[WIFI] AP 설정 완료. IP 주소: ");
  Serial.println(IP);

  // 1. 메인 웹페이지 
  server.on("/", HTTP_GET, [](){
    server.send_P(200, "text/html", index_html);
  });

  // 2. 상태 폴링 API
  server.on("/api/status", HTTP_GET, [](){
    char json[900];
    int sb = (digitalRead(LIMIT_BASE) == LIMIT_BASE_ACTIVE) ? 1 : 0;
    int ss = (digitalRead(LIMIT_SHOULDER) == LIMIT_SHOULDER_ACTIVE) ? 1 : 0;
    int se = (digitalRead(LIMIT_ELBOW) == LIMIT_ELBOW_ACTIVE) ? 1 : 0;
    int rp = (digitalRead(RELAY_VACUUM_PUMP) == RELAY_ON) ? 1 : 0;
    MPU6050_Data md = {0};
    if (xSemaphoreTake(mpuMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      md = mpu_data;
      xSemaphoreGive(mpuMutex);
    }
    CartesianPoint fk = solveFK();
    snprintf(json, sizeof(json), 
      "{\"state_idx\":%d,\"state_str\":\"%s\","
      "\"cur_pos\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f},"
      "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
      "\"freq\":%.2f,\"zeta\":%.3f,"
      "\"ptAU\":%d,\"ptAD\":%d,\"ptBU\":%d,\"ptBD\":%d,\"cycles\":%d,"
      "\"sw_base\":%d,\"sw_shoulder\":%d,\"sw_elbow\":%d,"
      "\"relay_pump\":%d,\"mpu_ok\":%d}",
      (int)currentState, stateStrings[currentState].c_str(),
      fk.x, fk.y, fk.z,
      md.ax, md.ay, md.az, md.gx, md.gy, md.gz,
      zvd_natural_freq_hz, zvd_damping_ratio,
      pointA_up.saved?1:0, pointA_down.saved?1:0, pointB_up.saved?1:0, pointB_down.saved?1:0, (int)cycleCount,
      sb, ss, se, rp, mpu_connected ? 1 : 0);
    server.send(200, "application/json", json);
  });

  // 3. 제어 명령 API
  server.on("/api/command", HTTP_POST, [](){
    if(server.hasArg("c")){
      String cmd = server.arg("c");
      if(cmd == "startCycle") webCmd_ContinuousCycle = true;
      else if(cmd == "saveAU") webCmd_SaveAU = true;
      else if(cmd == "saveAD") webCmd_SaveAD = true;
      else if(cmd == "saveBU") webCmd_SaveBU = true;
      else if(cmd == "saveBD") webCmd_SaveBD = true;
      else if(cmd == "rehome") webCmd_Rehome = true;
      else if(cmd == "tune") webCmd_TuneZVD = true;
      else if(cmd == "estop") {
        webCmd_EStop = true;
        webCmd_ContinuousCycle = false;  // 사이클 중단
        digitalWrite(RELAY_VACUUM_PUMP, RELAY_OFF);
        Serial.println("[CMD] E-STOP received!");
      }
    }
    server.send(200, "text/plain", "OK");
  });

  // 5. 하드웨어 수동 테스트 API (사이클 동작 중에는 무시)
  server.on("/api/test_cmd", HTTP_POST, [](){
    if (currentState != STATE_IDLE) {
      server.send(409, "text/plain", "BUSY");
      return;
    }
    if(server.hasArg("target") && server.hasArg("state")) {
      String target = server.arg("target");
      int state = server.arg("state").toInt();
      if(target == "pump") digitalWrite(RELAY_VACUUM_PUMP, state ? RELAY_ON : RELAY_OFF);
      else if(target.startsWith("jog_")) {
        int steps = 50;
        if(server.hasArg("steps")) steps = constrain(server.arg("steps").toInt(), 1, 5000);
        int pinStep, pinDir;
        volatile int32_t* stepCounter = nullptr;
        if(target == "jog_base")          { pinStep = MOTOR1_BASE_STEP; pinDir = MOTOR1_BASE_DIR; stepCounter = &currentSteps_base; }
        else if(target == "jog_shoulder") { pinStep = MOTOR2_SHOULDER_STEP; pinDir = MOTOR2_SHOULDER_DIR; stepCounter = &currentSteps_shoulder; }
        else if(target == "jog_elbow")    { pinStep = MOTOR3_ELBOW_STEP; pinDir = MOTOR3_ELBOW_DIR; stepCounter = &currentSteps_elbow; }
        else if(target == "jog_z") {
          // 자코비안 기반 수직 조그: 숫더+엘보 동시 구동으로 수직 이동
          float z_mm = (float)(steps * ((state > 0) ? 1 : -1));
          float sh_r = (float)currentSteps_shoulder / STEPS_PER_DEG_SHOULDER * (PI / 180.0f);
          float el_r = (float)currentSteps_elbow / STEPS_PER_DEG_ELBOW * (PI / 180.0f);
          float s1 = sinf(sh_r), c1 = cosf(sh_r);
          float s12 = sinf(sh_r + el_r), c12 = cosf(sh_r + el_r);
          // J = [dr/dθ1  dr/dθ2]   [-L1*s1-L2*s12  -L2*s12]
          //     [dz/dθ1  dz/dθ2] = [ L1*c1+L2*c12   L2*c12]
          float J11 = -LOW_SHANK_LENGTH*s1 - HIGH_SHANK_LENGTH*s12;
          float J12 = -HIGH_SHANK_LENGTH*s12;
          float J21 =  LOW_SHANK_LENGTH*c1 + HIGH_SHANK_LENGTH*c12;
          float J22 =  HIGH_SHANK_LENGTH*c12;
          if (fabsf(J11) < 0.01f) { server.send(400, "text/plain", "SING"); return; }
          float ratio = -J12 / J11;  // dθ1/dθ2 for dr=0
          float dz_per = J21 * ratio + J22;
          if (fabsf(dz_per) < 0.01f) { server.send(400, "text/plain", "SING"); return; }
          float dth2 = z_mm / dz_per;
          float dth1 = ratio * dth2;
          int32_t ds = (int32_t)(dth1 * STEPS_PER_DEG_SHOULDER * (180.0f / PI));
          int32_t de = (int32_t)(dth2 * STEPS_PER_DEG_ELBOW * (180.0f / PI));
          int32_t ts = currentSteps_shoulder + ds;
          int32_t te = currentSteps_elbow + de;
          Serial.printf("[JOG_Z] %.0fmm -> sh%+d el%+d\n", z_mm, (int)ds, (int)de);
          MotionCommand cmd = {currentSteps_base, ts, te, 1000, false, false};
          xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(500));
          server.send(200, "text/plain", "OK");
          return;
        }
        else { server.send(400, "text/plain", "BAD"); return; }
        
        
        int dir = (state > 0) ? 1 : -1;
        // Common Anode로 인해 DIR 논리 반전 (state > 0 이면 LOW).
        // GPIO 6/7 모터(엘보우)만 추가 반전 필요.
        if (target == "jog_elbow") {
          digitalWrite(pinDir, state > 0 ? HIGH : LOW);
        } else {
          digitalWrite(pinDir, state > 0 ? LOW : HIGH);
        }
        delayMicroseconds(5);
        for(int i = 0; i < steps; i++) {
          stepPulse(pinStep);
          delayMicroseconds(HOMING_SPEED_US);
          if (i > 0 && i % 50 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
        }


        // 스텝 카운터 업데이트 (FK 좌표 동기화)
        portENTER_CRITICAL(&stepsMux);
        *stepCounter += dir * steps;
        portEXIT_CRITICAL(&stepsMux);
      }
    }
    server.send(200, "text/plain", "OK");
  });

  // 6. XYZ 조그 API (비동기 큐잉)
  server.on("/api/jog_xyz", HTTP_POST, [](){
    if (currentState != STATE_IDLE) { server.send(409, "text/plain", "BUSY"); return; }
    if (server.hasArg("axis") && server.hasArg("dist")) {
      webCmd_JogXYZ_Axis = server.arg("axis").charAt(0);
      webCmd_JogXYZ_Dist = server.arg("dist").toFloat();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "BAD");
    }
  });

  server.begin();
  Serial.println("[WEB] 동기식 웹 서버 구동 완료 (Port 80)");
}

// ═══════════════════════════════════════════════════════════════════════════
// ██  섹션 10: setup() & loop()
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200); delay(1000);
  Serial.println("==========================================");
  Serial.println("  ESP32-S3 ZVD 픽앤플레이스 + Web GUI");
  Serial.println("==========================================");

  // 1. MPU 통신을 가장 먼저 시도하여 하드웨어 연결 상태 파악
  mpu6050_init();

  // 티칭 데이터 플래시에서 복원
  loadTeachPoints();

  // Common Anode 배선(PUL+가 5V)에서 3.3V 보드가 펄스를 확실히 끄기 위해 Open-Drain 필수
  pinMode(MOTOR1_BASE_STEP, OUTPUT_OPEN_DRAIN); pinMode(MOTOR1_BASE_DIR, OUTPUT_OPEN_DRAIN); pinMode(MOTOR1_BASE_ENA, OUTPUT_OPEN_DRAIN);
  pinMode(MOTOR2_SHOULDER_STEP, OUTPUT_OPEN_DRAIN); pinMode(MOTOR2_SHOULDER_DIR, OUTPUT_OPEN_DRAIN); pinMode(MOTOR2_SHOULDER_ENA, OUTPUT_OPEN_DRAIN);
  pinMode(MOTOR3_ELBOW_STEP, OUTPUT_OPEN_DRAIN); pinMode(MOTOR3_ELBOW_DIR, OUTPUT_OPEN_DRAIN); pinMode(MOTOR3_ELBOW_ENA, OUTPUT_OPEN_DRAIN);
  pinMode(LIMIT_BASE, INPUT_PULLUP); pinMode(LIMIT_SHOULDER, INPUT_PULLUP); pinMode(LIMIT_ELBOW, INPUT_PULLUP);
  // 5V 릴레이와 3.3V ESP32의 전압 충돌(1.7V 잔류)을 막기 위해 Open-Drain 사용
  pinMode(RELAY_VACUUM_PUMP, OUTPUT_OPEN_DRAIN);
  
  // 모터 활성화 (ENA LOW) 및 릴레이 OFF
  motorsEnable();
  digitalWrite(RELAY_VACUUM_PUMP, RELAY_OFF);

  hardware_bypassed = false;
  Serial.println("[INIT] Hardware bypassed mode OFF -> real homing enabled");
  // 3. 웹 서버 실행
  setupWiFiAndWeb();

  // 4. 태스크 및 큐 생성
  motionQueue = xQueueCreate(MOTION_QUEUE_SIZE, sizeof(MotionCommand));
  motionDoneSem = xSemaphoreCreateBinary();
  mpuMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskStepper, "TaskStepper", TASK_STEPPER_STACK, NULL, TASK_STEPPER_PRIORITY, &taskStepperHandle, 1);
  xTaskCreatePinnedToCore(taskControl, "TaskControl", TASK_CONTROL_STACK, NULL, TASK_CONTROL_PRIORITY, &taskControlHandle, 0);
}

void loop() {
  server.handleClient();
  // MPU 읽기 (웹서버와 같은 Core에서 매 루프마다 실행 - 호밍 블로킹과 무관)
  if (mpu_connected) {
    if (xSemaphoreTake(mpuMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      mpu_data = mpu6050_read();
      xSemaphoreGive(mpuMutex);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
