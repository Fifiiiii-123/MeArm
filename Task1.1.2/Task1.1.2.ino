 /*
 * ============================================================================
 *  Task1.1(2) 基础控制·极简版（考核方向内 5%）
 * ----------------------------------------------------------------------------
 *  仅保留：摇杆实时操控 + 各关节独立控制
 *  （4 个摇杆轴各管一个关节，动哪个轴只动那个关节）
 *  已精简掉：串口 b/r/f/c 指令、回中、状态查询
 *
 *  摇杆映射（方向按 test1.1 实测校准）：
 *    右摇杆 上下=A3 → 后臂(D8)   右摇杆 左右=A2 → 底座(D9)
 *    左摇杆 上下=A1 → 前臂(D7)   左摇杆 左右=A0 → 夹爪(D6)
 *  舵机引脚：base=9  rArm=8  fArm=7  claw=6
 * ============================================================================
 */
#include <Servo.h>

/* ---------------- 引脚 ---------------- */
#define PIN_BASE  9
#define PIN_RARM  8
#define PIN_FARM  7
#define PIN_CLAW  6

#define JOY1_X    A3     // 右摇杆上下 → 后臂
#define JOY1_Y    A2     // 右摇杆左右 → 底座
#define JOY2_X    A1     // 左摇杆上下 → 前臂
#define JOY2_Y    A0     // 左摇杆左右 → 夹爪

/* ---------------- 限位（保护舵机） ---------------- */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  30          // 后臂角度下限
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  130         // 前臂角度上限
#define CLAW_MIN  5           // 夹爪张开极限角度（O 指令的目标）
#define CLAW_MAX  100         // 夹爪闭合极限角度（S 指令的目标）


/* ---------------- 摇杆参数 ---------------- */
#define JOY_CENTER   512   // 摇杆中点
#define JOY_DEADBAND 25    // 死区（中位 ±25 内不动）
#define JOY_MAX_STEP 1     // 单轮最大角度增量

/* ---------------- 方向系数（按 test1.1 实测：-1=反向） ---------------- */
#define DIR_JOY1_X  +1     // 后臂
#define DIR_JOY1_Y  -1     // 底座
#define DIR_JOY2_X  -1     // 前臂
#define DIR_JOY2_Y  +1     // 夹爪

Servo base, rArm, fArm, claw;
int baseAng = 90, rArmAng = 90, fArmAng = 90, clawAng = 90;

/* 角度限位 */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* 摇杆轴增量：死区内 0，越推越远 ±1~±JOY_MAX_STEP */
int axisDelta(int pin){
  int v = analogRead(pin);
  if (v > JOY_CENTER + JOY_DEADBAND)
    return map(v - (JOY_CENTER + JOY_DEADBAND), 0, 1023 - (JOY_CENTER + JOY_DEADBAND), 1, JOY_MAX_STEP);
  if (v < JOY_CENTER - JOY_DEADBAND)
    return -map((JOY_CENTER - JOY_DEADBAND) - v, 0, JOY_CENTER - JOY_DEADBAND, 1, JOY_MAX_STEP);
  return 0;
}

void setup(){
  base.attach(PIN_BASE);  delay(200);
  rArm.attach(PIN_RARM);  delay(200);
  fArm.attach(PIN_FARM);  delay(200);
  claw.attach(PIN_CLAW);  delay(200);
  base.write(85); rArm.write(90); fArm.write(75); claw.write(90);
  delay(300);
}

void loop(){
  /* 每个摇杆轴独立控制对应关节（方向系数见配置） */
  int dBase = DIR_JOY1_Y * axisDelta(JOY1_Y);
  int dRArm = DIR_JOY1_X * axisDelta(JOY1_X);
  int dFArm = DIR_JOY2_X * axisDelta(JOY2_X);
  int dClaw = DIR_JOY2_Y * axisDelta(JOY2_Y);

  if (dBase){ baseAng = clampVal(baseAng + dBase, BASE_MIN, BASE_MAX); base.write(baseAng); }
  if (dRArm){ rArmAng = clampVal(rArmAng + dRArm, RARM_MIN, RARM_MAX); rArm.write(rArmAng); }
  if (dFArm){ fArmAng = clampVal(fArmAng + dFArm, FARM_MIN, FARM_MAX); fArm.write(fArmAng); }
  if (dClaw){ clawAng = clampVal(clawAng + dClaw, CLAW_MIN, CLAW_MAX); claw.write(clawAng); }
  delay(15);
}
