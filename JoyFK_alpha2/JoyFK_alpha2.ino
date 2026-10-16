/*
 * ============================================================================
 *  文件：JoyFK.ino
 *  工程：JoyFK —— 摇杆控制 + 运动学正解实时显示坐标（简洁版）
 * ----------------------------------------------------------------------------
 *  功能：
 *    只用两个摇杆直接控制 4 个关节角度（底座/上臂/前臂/夹爪），
 *    每 250ms 用运动学正解算出腕点坐标 (x, y, z) 并串口打印，
 *    用于验证任务四的运动学正解公式。
 *
 *  摇杆映射（与 Task3 实测方向一致）：
 *    右摇杆上下 A3 → 上臂仰角 α      右摇杆左右 A2 → 底座转角 θ1
 *    左摇杆上下 A1 → 前臂内角 β      左摇杆左右 A0 → 夹爪
 *
 *  运动学正解（按用户手写图定稿，β = L2 与 L3 的肘关节内角，前臂下垂模型）：
 *    r  = L2·cosα − L3·cos(α+β)
 *    mx = r·sinθ1，my = r·cosθ1（θ1 从 Y 轴量起，θ1=0 时小臂沿 Y 延伸）
 *    mz = L1 + L2·sinα − L3·sin(α+β)（L3 项为 −：β<90 前臂下垂、mz 减小）
 *
 *  硬件接线：
 *    舵机：base=D9  rArm=D8  fArm=D7  claw=D6
 *    摇杆：A3/A2/A1/A0（ArmJoystick V3.0）
 *  尺寸：L1=38mm（底盘高度），L2=80mm（上臂），L3=80mm（前臂）
 * ============================================================================
 */
#include <Servo.h>
#include <math.h>

/* ---------------- 引脚定义 ---------------- */
#define PIN_BASE  9           // 底座舵机
#define PIN_RARM  8           // 上臂舵机
#define PIN_FARM  7           // 前臂舵机
#define PIN_CLAW  6           // 夹爪舵机

/* ---------------- 摇杆模拟引脚 ---------------- */
#define JOY1_X    A3          // 右摇杆上下 → 上臂 α
#define JOY1_Y    A2          // 右摇杆左右 → 底座 θ1
#define JOY2_X    A1          // 左摇杆上下 → 前臂 β
#define JOY2_Y    A0          // 左摇杆左右 → 夹爪

/* ---------------- 舵机限位（借鉴 Task3） ---------------- */
#define BASE_MIN  0
#define BASE_MAX  180
#define RARM_MIN  45
#define RARM_MAX  160
#define FARM_MIN  30
#define FARM_MAX  110         // 前臂若只能下垂，建议改为 90（见 servoToGeo 注释）
#define CLAW_MIN  5
#define CLAW_MAX  130

/* ---------------- 机械臂尺寸（mm，按实物） ---------------- */
#define L1  38.0f    // 底盘高度：底座顶面 → 肩轴
#define L2  80.0f    // 上臂长度：肩 → 肘
#define L3  80.0f    // 前臂长度：肘 → 腕

/* ---------------- 摇杆参数 ---------------- */
#define JOY_CENTER   512      // 摇杆模拟值中点
#define JOY_DEADBAND 25       // 死区（中位 ±25 内不动）
#define JOY_MAX_STEP 1        // 单轮最大角度增量

/* ---------------- 摇杆方向配置（与 Task1.1 实测一致） ----------------
 * 摇杆推杆方向由硬件固定，关节正方向由舵机安装决定；
 * 实测 3 个轴推杆方向与期望运动相反故取反，夹爪恰好一致为 +1。
 *   右摇杆上下=A3 → 上臂α(-1)    右摇杆左右=A2 → 底座θ1(-1)
 *   左摇杆上下=A1 → 前臂β(-1)    左摇杆左右=A0 → 夹爪(+1) */
#define DIR_JOY1_X  -1         // 上臂方向系数
#define DIR_JOY1_Y  -1         // 底座方向系数
#define DIR_JOY2_X  +1         // 前臂方向系数
#define DIR_JOY2_Y  +1         // 夹爪方向系数

/* ---------------- 运动速度（与其他程序一致） ----------------
 * servoDelay 是"每转动 1° 的延时毫秒数"：延时越大转得越慢。
 * 摇杆操控时每步延时用 servoDelay，手感与 Task1.1/Task3 一致。 */
int servoDelay = 20;          // 默认每度延时 15ms（速度适中）

/* ---------------- 打印周期 ---------------- */
#define PRINT_MS  250         // 每 250ms 串口打印一次坐标

/* ---------------- 舵机对象与当前角度 ---------------- */
Servo base, rArm, fArm, claw;
int curBase = 90, curRArm = 90, curFArm = 90, curClaw = 90;

/* ==================== 角度限位钳制 ==================== */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 摇杆轴增量 ====================
 * 死区内返回 0，越推越远返回越大（±1 ~ ±JOY_MAX_STEP） */
int axisDelta(int pin){
  int v = analogRead(pin);
  if (v > JOY_CENTER + JOY_DEADBAND){
    int span = 1023 - (JOY_CENTER + JOY_DEADBAND);
    if (span <= 0) return 1;
    return map(v - (JOY_CENTER + JOY_DEADBAND), 0, span, 1, JOY_MAX_STEP);
  }
  if (v < JOY_CENTER - JOY_DEADBAND){
    int span = JOY_CENTER - JOY_DEADBAND;
    if (span <= 0) return -1;
    return -map((JOY_CENTER - JOY_DEADBAND) - v, 0, span, 1, JOY_MAX_STEP);
  }
  return 0;
}

/* ==================== 舵机角 → 几何角（度） ====================
 * 默认标定（按实物修正时只改这里）：
 *   底座：90° = 正前方 → θ1 = curBase − 90
 *   上臂：α = 180 − curRArm（实测方向相反：大臂前摆时 curRArm 增大、α 应减小；
 *         故取反：curRArm=90→α=90 竖直，curRArm 增大→α 减小→前倾 mz 下降）
 *   前臂：β = curFArm（fArm=90° 时前臂水平前伸、与上臂成 90°；
 *         归中时水平投影 = 前臂长度 L3，符合实物；
 *         fArm 减小→β<90→前臂上翘 mz 增大；fArm 增大→β>90→前臂下垂 mz 减小）
 * 若实物前臂只能下垂，β 应 ≤ 90°，即 curFArm ≤ 90，建议 FARM_MAX 改 90。 */
void servoToGeo(float &t1, float &alpha, float &beta){
  t1    = (float)(curBase - 90);
  alpha = (float)(180 - curRArm);
  beta  = (float)curFArm;
}

/* ==================== 运动学正解 ====================
 * 输入：θ1（底座转角，从 Y 轴量起）、α（上臂仰角）、β（L2 与 L3 的肘关节内角），单位度
 * 输出：mx、my、mz（mm，原点=底座转轴与地面交点）
 * 公式（按用户手写图定稿，β 内角·前臂下垂模型；θ1 从 Y 轴量起）：
 *   r  = L2·cosα − L3·cos(α+β)
 *   mx = r·sinθ1，my = r·cosθ1（θ1=0 时腕点在 Y 方向，小臂沿 Y 延伸）
 *   mz = L1 + L2·sinα − L3·sin(α+β)   ← L3 项为 −：β<90 前臂下垂、mz 减小 */
void forwardKin(float t1Deg, float aDeg, float bDeg,
                float &mx, float &my, float &mz){
  float t1 = radians(t1Deg);
  float a  = radians(aDeg);
  float b  = radians(bDeg);
  float r  = L2 * cos(a) - L3 * cos(a + b);   // 腕点水平投影距离
  mx = r * sin(t1);                            // X 方向坐标（θ1=0 时为 0）
  my = r * cos(t1);                            // Y 方向坐标（θ1=0 时 = r，小臂沿 Y 延伸）
  mz = L1 + L2 * sin(a) - L3 * sin(a + b);     // 高度坐标（− 号：β 内角，前臂下垂）
}

/* ==================== 串口打印当前坐标 ==================== */
void printPos(){
  float t1, a, b, mx, my, mz;
  servoToGeo(t1, a, b);
  forwardKin(t1, a, b, mx, my, mz);
  Serial.print("pos mm: x="); Serial.print(mx, 1);
  Serial.print("  y=");      Serial.print(my, 1);
  Serial.print("  z=");      Serial.print(mz, 1);
  Serial.print("  | joints: curBase="); Serial.print(curBase);
  Serial.print("  alpha=");     Serial.print(180 - curRArm);
  Serial.print("  curFArm=");   Serial.print(curFArm);
  Serial.print("  claw=");   Serial.println(curClaw);
}

/* ==================== 初始化 ==================== */
void setup(){
  base.attach(PIN_BASE);  delay(200);
  rArm.attach(PIN_RARM);  delay(200);
  fArm.attach(PIN_FARM);  delay(200);
  claw.attach(PIN_CLAW);  delay(200);

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90);
  delay(300);

  Serial.begin(9600);
  Serial.println("=== JoyFK: 摇杆控制 + 运动学正解显示坐标 ===");
  Serial.println("右摇杆: 上下=上臂α  左右=底座θ1 | 左摇杆: 上下=前臂β  左右=夹爪");
  Serial.println("L1=38mm  L2=80mm  L3=80mm");
}

/* ==================== 主循环 ====================
 * 每轮：读摇杆 → 更新关节角（限位）→ 驱动舵机 → 周期打印坐标
 * 摇杆为"速度控制"（与 Task1.1 一致）：摇杆回中不动，
 * 有输入时每步延时 servoDelay，推杆越远转动越快（见 axisDelta）。 */
void loop(){
  int dT1   = DIR_JOY1_Y * axisDelta(JOY1_Y);   // 右摇杆左右 → 底座
  int dA    = DIR_JOY1_X * axisDelta(JOY1_X);   // 右摇杆上下 → 上臂
  int dB    = DIR_JOY2_X * axisDelta(JOY2_X);   // 左摇杆上下 → 前臂
  int dClaw = DIR_JOY2_Y * axisDelta(JOY2_Y);   // 左摇杆左右 → 夹爪

  if (dT1 || dA || dB || dClaw){                // 任一摇杆轴有输入（与 Task1.1 一致）
    if (dT1)  { curBase = clampVal(curBase + dT1,  BASE_MIN, BASE_MAX);  base.write(curBase); }
    if (dA)   { curRArm = clampVal(curRArm + dA,   RARM_MIN, RARM_MAX);  rArm.write(curRArm); }
    if (dB)   { curFArm = clampVal(curFArm + dB,   FARM_MIN, FARM_MAX);  fArm.write(curFArm); }
    if (dClaw){ curClaw = clampVal(curClaw + dClaw, CLAW_MIN, CLAW_MAX); claw.write(curClaw); }
    delay(servoDelay);                          // 每步延时 = 摇杆操控速度（与 Task1.1 一致）
  }

  static unsigned long lastPrint = 0;         // 周期打印坐标
  if (millis() - lastPrint >= PRINT_MS){
    lastPrint = millis();
    printPos();
  }
}
