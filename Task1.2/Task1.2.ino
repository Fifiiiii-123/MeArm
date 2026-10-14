/*
 * ============================================================================
 *  文件：Task1.2.ino
 *  工程：Task1.2 —— 机械臂基本功能 · 固定指令通信（考核方向内 5%）
 * ----------------------------------------------------------------------------
 *  考核点：上位机（电脑）串口发送单字符指令，机械臂正确响应
 *     O → 机械臂爪子张开
 *     S → 机械臂爪子闭合
 *     H → 提升机械臂整体运行速度
 *     L → 降低机械臂整体运行速度
 *  （字母大小写均可）
 *
 *  功能说明：
 *    1. O/S 控制夹爪开合（经同步移动函数平滑执行）
 *    2. H/L 调整"每度延时"servoDelay：H 减小延时=变快，L 增大延时=变慢，
 *       所有关节的移动速度同步变化（评分点：速度档作用于整体运行速度）
 *    3. C 回中、? 状态（辅助演示功能，不影响考核点）
 *
 *  串口协议（9600 波特率，指令以回车/换行结束）：
 *        O  夹爪张开（转到张开极限角度 CLAW_MIN）
 *        S  夹爪闭合（转到闭合极限角度 CLAW_MAX）
 *        H  提速：servoDelay 减小 SPEED_STEP（下限 DELAY_MIN）
 *        L  降速：servoDelay 增大 SPEED_STEP（上限 DELAY_MAX）
 *        C  回中（90/90/90/夹爪30）
 *        ?  打印当前状态（含速度档）
 *
 *  硬件接线：
 *        舵机：base=D9  rArm=D8  fArm=D7  claw=D6（信号线）
 *        本任务无需摇杆，仅用串口演示
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库（标准库）

/* ---------------- 舵机引脚定义 ----------------
 * 每个舵机占用一个数字引脚，通过 PWM 信号控制角度。 */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6

/* ---------------- 舵机限位 ----------------
 * 每个关节的机械行程限制，程序内强制钳制到安全范围。
 * 夹爪：CLAW_MIN=张开极限，CLAW_MAX=闭合极限；
 * 若夹爪方向相反，交换 CLAW_MIN 与 CLAW_MAX 两个值。 */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  30          // 后臂角度下限
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  130         // 前臂角度上限
#define CLAW_MIN  5           // 夹爪张开极限角度（O 指令的目标）
#define CLAW_MAX  100         // 夹爪闭合极限角度（S 指令的目标）


/* ---------------- 速度档参数 ----------------
 * 速度档由 servoDelay 体现：
 *   H 每次把 servoDelay 减去 SPEED_STEP，但不能低于 DELAY_MIN（最快）
 *   L 每次把 servoDelay 加上 SPEED_STEP，但不能高于 DELAY_MAX（最慢）
 * 速度变化后，所有关节的移动速度同步改变（syncMove 共用该值）。 */
int servoDelay = 15;          // 每转 1° 的延时毫秒数，越大越慢
#define DELAY_MIN   3         // 最快档：每度延时下限 3ms
#define DELAY_MAX   50        // 最慢档：每度延时上限 50ms
#define SPEED_STEP  5         // 每次 H/L 改变的速度档步长（5ms）

/* ---------------- 舵机对象与当前角度 ----------------
 * 全局记录当前角度，运动基于当前值逐度逼近。 */
Servo base, rArm, fArm, claw; // 4 个舵机对象
int curBase = 90;             // 底座当前角度
int curRArm = 90;             // 后臂当前角度
int curFArm = 90;             // 前臂当前角度
int curClaw = 90;             // 夹爪当前角度

/* ---------------- 串口行缓冲 ---------------- */
char lineBuf[32];             // 一行指令的字符缓冲
int  lineLen = 0;             // 缓冲中已接收的字符数

/* ==================== 角度限位钳制 ====================
 * 功能：把角度 v 限制在 [lo, hi] 范围内并返回 */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 多舵机同步移动 ====================
 * 功能：4 个关节同步逐度运动到目标角度
 * 参数：tBase/tRArm/tFArm/tClaw 目标角度；stepMs 每步延时（即速度档）
 * 说明：所有关节共用 stepMs → H/L 改变 servoDelay 即可同时改变
 *       所有关节的移动速度，实现"整体运行速度"的调整 */
void syncMove(int tBase, int tRArm, int tFArm, int tClaw, int stepMs){
  tBase = clampVal(tBase, BASE_MIN, BASE_MAX);      // 底座限位
  tRArm = clampVal(tRArm, RARM_MIN, RARM_MAX);      // 后臂限位
  tFArm = clampVal(tFArm, FARM_MIN, FARM_MAX);      // 前臂限位
  tClaw = clampVal(tClaw, CLAW_MIN, CLAW_MAX);      // 夹爪限位

  bool done = false;                                // 是否全部到位
  while (!done){
    done = true;
    if (curBase != tBase){ curBase += (tBase > curBase) ? 1 : -1; base.write(curBase);  done = false; }
    if (curRArm != tRArm){ curRArm += (tRArm > curRArm) ? 1 : -1; rArm.write(curRArm); done = false; }
    if (curFArm != tFArm){ curFArm += (tFArm > curFArm) ? 1 : -1; fArm.write(curFArm); done = false; }
    if (curClaw != tClaw){ curClaw += (tClaw > curClaw) ? 1 : -1; claw.write(curClaw); done = false; }
    delay(stepMs);                                  // 每步延时 = 当前速度档
  }
}

/* ==================== O：爪子张开 ====================
 * 功能：夹爪转动到张开极限角度 CLAW_MIN，其余关节保持不动 */
void clawOpen(){
  Serial.println("+ 爪子张开 (O)");
  syncMove(curBase, curRArm, curFArm, CLAW_MIN, servoDelay);
}

/* ==================== S：爪子闭合 ====================
 * 功能：夹爪转动到闭合极限角度 CLAW_MAX，其余关节保持不动 */
void clawClose(){
  Serial.println("+ 爪子闭合 (S)");
  syncMove(curBase, curRArm, curFArm, CLAW_MAX, servoDelay);
}

/* ==================== H：提升速度 ====================
 * 功能：每度延时减小 SPEED_STEP（下限 DELAY_MIN），整体移动变快 */
void speedUp(){
  servoDelay = clampVal(servoDelay - SPEED_STEP, DELAY_MIN, DELAY_MAX);
  Serial.print("+ 速度提升(H)：每度延时 "); Serial.print(servoDelay); Serial.println(" ms");
}

/* ==================== L：降低速度 ====================
 * 功能：每度延时增大 SPEED_STEP（上限 DELAY_MAX），整体移动变慢 */
void speedDown(){
  servoDelay = clampVal(servoDelay + SPEED_STEP, DELAY_MIN, DELAY_MAX);
  Serial.print("+ 速度降低(L)：每度延时 "); Serial.print(servoDelay); Serial.println(" ms");
}

/* ==================== C：回中 ====================
 * 功能：所有关节回到初始姿态（90/90/90/夹爪70） */
void centerAll(){
  Serial.println(">>> 回中");
  syncMove(90, 90, 90, 70, servoDelay);
}

/* ==================== 状态报告 ====================
 * 功能：打印各关节角度和当前速度档，便于观察 H/L 的效果 */
void reportStatus(){
  Serial.println("----- 机械臂状态 -----");
  Serial.print("底座="); Serial.print(curBase);
  Serial.print(" 后臂="); Serial.print(curRArm);
  Serial.print(" 前臂="); Serial.print(curFArm);
  Serial.print(" 夹爪="); Serial.println(curClaw);
  Serial.print("速度档（每度延时）："); Serial.println(servoDelay);
  Serial.println("------------------------");
}

/* ==================== 单字符指令解析 ====================
 * 功能：处理一条串口指令（仅单字符）
 * 支持：O/S/H/L/C/?，大小写均可（toupper 统一转大写后匹配）
 * 说明：本任务专注"固定指令通信"考核点，不接收多字符指令 */
void handleCommand(const char *line){
  int len = strlen(line);                           // 指令长度
  if (len == 0) return;                             // 空指令忽略
  char c = toupper(line[0]);                        // 取首字符转大写

  if (len == 1){
    switch (c){                                     // 按指令分发
      case 'O': clawOpen(); break;                  // 夹爪张开
      case 'S': clawClose(); break;                 // 夹爪闭合
      case 'H': speedUp(); break;                   // 提升速度
      case 'L': speedDown(); break;                 // 降低速度
      case 'C': centerAll(); break;                 // 回中（辅助）
      case '?': reportStatus(); break;              // 状态（辅助）
      default:  Serial.println("! 未知指令（O/S/H/L/C/?）"); break; // 非法指令
    }
    return;
  }
  Serial.println("! 本任务仅支持单字符指令");         // 多余字符提示
}

/* ==================== 串口逐行读取 ====================
 * 功能：非阻塞读取串口，拼成完整一行后调用 handleCommand */
void readSerialLine(){
  while (Serial.available()){
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r'){                  // 行结束
      if (lineLen > 0){
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < (int)sizeof(lineBuf) - 1){
      lineBuf[lineLen++] = ch;
    } else {
      lineLen = 0;                                  // 溢出丢弃
    }
  }
}

/* ==================== 初始化 ==================== */
void setup(){
  base.attach(PIN_BASE);  delay(200);               // 底座舵机接上引脚 9
  rArm.attach(PIN_RARM);  delay(200);               // 后臂舵机接上引脚 8
  fArm.attach(PIN_FARM);  delay(200);               // 前臂舵机接上引脚 7
  claw.attach(PIN_CLAW);  delay(200);               // 夹爪舵机接上引脚 6

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90); // 初始 90°
  delay(300);                                       // 等舵机到位

  Serial.begin(9600);                               // 串口 9600
  Serial.println("=== Task1.2 固定指令通信：O/S/H/L ===");
  Serial.println("O 张开 | S 闭合 | H 提速 | L 减速 | C 回中 | ? 状态");
}

/* ==================== 主循环 ====================
 * 每轮只做一件事：等待并处理串口指令 */
void loop(){
  readSerialLine();
}
