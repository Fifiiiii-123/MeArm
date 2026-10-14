/*
 * ============================================================================
 *  文件：Task1.ino
 *  工程：Task1 —— 任务一整合版（1.1 基础控制 + 1.2 固定指令通信 + 1.3 多舵机协同）
 * ----------------------------------------------------------------------------
 *  本工程把任务一的三个考核点合并到同一个程序：
 *
 *  ① Task1.1 基础控制（5%）
 *      - 摇杆实时操控 4 个关节（速度控制 + 死区）
 *      - 串口单关节独立移动：b角度/r角度/f角度/c角度
 *  ② Task1.2 固定指令通信（5%）
 *      - O 夹爪张开 / S 夹爪闭合
 *      - H 提升速度 / L 降低速度（改变整体运行速度）
 *  ③ Task1.3 多舵机协同（5%）
 *      - x角度,y角度,z角度 单条指令同步控制三个舵机
 *
 *  串口协议（9600 波特率，指令以回车/换行结束，字母大小写均可）：
 *        b角度  底座转到指定角度（如 b90）
 *        r角度  后臂转到指定角度（如 r60）
 *        f角度  前臂转到指定角度（如 f45）
 *        c角度  夹爪转到指定角度（如 c30）
 *        O      夹爪张开到极限（CLAW_MIN）
 *        S      夹爪闭合到极限（CLAW_MAX）
 *        H      提升机械臂运行速度
 *        L      降低机械臂运行速度
 *        C      所有关节回到初始姿态（90/90/90/30）
 *        ?      打印当前各关节角度
 *        x,y,z  三舵机协同（例：x10,y30,z20 → 底座/后臂/前臂同步转动）
 *
 *  摇杆通道映射（按 test1.1 实测校准）：
 *        右摇杆 上下=A3 → 后臂(D8)（方向取反）
 *        右摇杆 左右=A2 → 底座(D9)（方向取反）
 *        左摇杆 上下=A1 → 前臂(D7)（方向取反）
 *        左摇杆 左右=A0 → 夹爪(D6)（不取反）
 *
 *  硬件接线：
 *        舵机：base=D9  rArm=D8  fArm=D7  claw=D6（信号线）
 *        摇杆：A3/A2/A1/A0（模拟输入）
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库（标准库）

/* ---------------- 舵机引脚定义 ---------------- */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6

/* ---------------- 摇杆模拟引脚定义 ---------------- */
#define JOY1_X    A3          // 右摇杆上下轴 → 后臂
#define JOY1_Y    A2          // 右摇杆左右轴 → 底座
#define JOY2_X    A1          // 左摇杆上下轴 → 前臂
#define JOY2_Y    A0          // 左摇杆左右轴 → 夹爪

/* ---------------- 舵机限位（超限自动钳制，保护舵机） ---------------- */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  30          // 后臂角度下限
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  130         // 前臂角度上限
#define CLAW_MIN  5           // 夹爪张开极限角度（O 指令的目标）
#define CLAW_MAX  100         // 夹爪闭合极限角度（S 指令的目标）

/* ---------------- 摇杆控制参数 ----------------
 * 摇杆是"速度控制"而非"位置跟随"：推杆越远离中位，关节转动越快。 */
#define JOY_CENTER   512      // 摇杆模拟值中点（未拨动时的读值）
#define JOY_DEADBAND 25       // 死区：中位 ±25 内视为"未拨动"，防止误触抖动
#define JOY_MAX_STEP 1        // 单轮循环最大角度增量（度）；值越大越灵敏

/* ---------------- 摇杆方向配置（按 test1.1 实测） ----------------
 * 摇杆推杆方向由硬件固定，关节正方向由舵机安装决定；
 * 按实测以下 3 轴推杆方向与期望运动相反 → 系数 -1（取反），夹爪 +1。 */
#define DIR_JOY1_X  -1         // 后臂方向系数
#define DIR_JOY1_Y  -1         // 底座方向系数
#define DIR_JOY2_X  -1         // 前臂方向系数
#define DIR_JOY2_Y  +1         // 夹爪方向系数

/* ---------------- 速度档参数 ----------------
 * servoDelay = 每转动 1° 的延时毫秒数，越大越慢。
 * H 每次减小 SPEED_STEP（下限 DELAY_MIN=最快），L 每次增大（上限 DELAY_MAX=最慢）。
 * 速度变化作用于 syncMove，即所有关节整体运行速度同步改变。 */
int servoDelay = 15;          // 默认每度延时 15ms（速度适中）
#define DELAY_MIN   3         // 最快档：每度延时下限 3ms
#define DELAY_MAX   70        // 最慢档：每度延时上限 70ms
#define SPEED_STEP  5         // 每次 H/L 改变的速度档步长（5ms）

/* ---------------- 舵机对象与当前角度 ----------------
 * 全局记录"当前实际角度"，所有运动基于当前值逐度逼近，
 * 串口指令、摇杆操作、协同控制可以无缝叠加，不会出现角度跳变。 */
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

/* ==================== 摇杆轴增量读取 ====================
 * 功能：读取某摇杆轴偏离中位的程度，换算成角度增量
 * 返回：死区内 0；越往一端推 ±1~±JOY_MAX_STEP（符号=方向） */
int axisDelta(int pin){
  int v = analogRead(pin);                          // 读取模拟值 0~1023
  if (v > JOY_CENTER + JOY_DEADBAND){               // 正向推过了死区
    int span = 1023 - (JOY_CENTER + JOY_DEADBAND);  // 可用正向行程
    if (span <= 0) return 1;                        // 防御：行程异常时最小步进
    return map(v - (JOY_CENTER + JOY_DEADBAND), 0, span, 1, JOY_MAX_STEP);
  }
  if (v < JOY_CENTER - JOY_DEADBAND){               // 反向推过了死区
    int span = JOY_CENTER - JOY_DEADBAND;           // 可用反向行程
    if (span <= 0) return -1;
    return -map((JOY_CENTER - JOY_DEADBAND) - v, 0, span, 1, JOY_MAX_STEP);
  }
  return 0;                                         // 在死区内：不动
}

/* ==================== 多舵机同步移动 ====================
 * 功能：4 个关节"同时、逐度"运动到目标角度
 * 说明：
 *   ① 目标角度先限位钳制，超限自动截断并提示
 *   ② while 循环每轮每个关节走 1°，全部到位才结束
 *   ③ 单关节独立移动 = 只传一个目标角度，其余传当前值 → 自然保持不动 */
void syncMove(int tBase, int tRArm, int tFArm, int tClaw, int stepMs){
  bool outOfLimit = false;                          // 是否有角度超限
  if (tBase < BASE_MIN || tBase > BASE_MAX){ tBase = clampVal(tBase, BASE_MIN, BASE_MAX); outOfLimit = true; }
  if (tRArm < RARM_MIN || tRArm > RARM_MAX){ tRArm = clampVal(tRArm, RARM_MIN, RARM_MAX); outOfLimit = true; }
  if (tFArm < FARM_MIN || tFArm > FARM_MAX){ tFArm = clampVal(tFArm, FARM_MIN, FARM_MAX); outOfLimit = true; }
  if (tClaw < CLAW_MIN || tClaw > CLAW_MAX){ tClaw = clampVal(tClaw, CLAW_MIN, CLAW_MAX); outOfLimit = true; }
  if (outOfLimit) Serial.println("! 部分目标角度超出限位，已自动限制");

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

/* ==================== 单关节独立移动（Task1.1） ====================
 * 功能：只移动指定关节到目标角度，其余关节保持不动 */
void moveBase (int t){ syncMove(t,          curRArm, curFArm, curClaw, servoDelay); } // 底座
void moveRArm (int t){ syncMove(curBase, t,          curFArm, curClaw, servoDelay); } // 后臂
void moveFArm (int t){ syncMove(curBase, curRArm, t,          curClaw, servoDelay); } // 前臂
void moveClaw (int t){ syncMove(curBase, curRArm, curFArm, t,          servoDelay); } // 夹爪

/* ==================== O：爪子张开（Task1.2） ====================
 * 功能：夹爪转动到张开极限角度 CLAW_MIN，其余关节保持不动 */
void clawOpen(){
  Serial.println("+ 爪子张开 (O)");
  syncMove(curBase, curRArm, curFArm, CLAW_MIN, servoDelay);
}

/* ==================== S：爪子闭合（Task1.2） ====================
 * 功能：夹爪转动到闭合极限角度 CLAW_MAX，其余关节保持不动 */
void clawClose(){
  Serial.println("+ 爪子闭合 (S)");
  syncMove(curBase, curRArm, curFArm, CLAW_MAX, servoDelay);
}

/* ==================== H：提升速度（Task1.2） ====================
 * 功能：每度延时减小 SPEED_STEP（下限 DELAY_MIN），整体移动变快 */
void speedUp(){
  servoDelay = clampVal(servoDelay - SPEED_STEP, DELAY_MIN, DELAY_MAX);
  Serial.print("+ 速度提升(H)：每度延时 "); Serial.print(servoDelay); Serial.println(" ms");
}

/* ==================== L：降低速度（Task1.2） ====================
 * 功能：每度延时增大 SPEED_STEP（上限 DELAY_MAX），整体移动变慢 */
void speedDown(){
  servoDelay = clampVal(servoDelay + SPEED_STEP, DELAY_MIN, DELAY_MAX);
  Serial.print("+ 速度降低(L)：每度延时 "); Serial.print(servoDelay); Serial.println(" ms");
}

/* ==================== C：回中 ====================
 * 功能：所有关节回到初始姿态（90/90/90/夹爪30） */
void centerAll(){
  Serial.println(">>> 回中");
  syncMove(90, 90, 90, 30, servoDelay);
}

/* ==================== 状态报告 ====================
 * 功能：打印各关节角度和当前速度档 */
void reportStatus(){
  Serial.println("----- 机械臂状态 -----");
  Serial.print("底座="); Serial.print(curBase);
  Serial.print(" 后臂="); Serial.print(curRArm);
  Serial.print(" 前臂="); Serial.print(curFArm);
  Serial.print(" 夹爪="); Serial.println(curClaw);
  Serial.print("速度档（每度延时）："); Serial.println(servoDelay);
  Serial.println("------------------------");
}

/* ==================== 数字解析 ====================
 * 功能：从字符串 s 的 idx 位置开始解析一个十进制数字
 * 返回：解析到的整数；该位置没有数字返回 -1 */
int getNumberAfter(const char *s, int idx){
  int v = 0;                                        // 累加结果
  bool any = false;                                 // 是否解析到数字
  while (s[idx] == ' ') idx++;                      // 跳过前导空格
  while (s[idx] >= '0' && s[idx] <= '9'){           // 逐位累加数字
    v = v * 10 + (s[idx] - '0');
    any = true;
    idx++;
  }
  return any ? v : -1;                              // 有数字返回数值，否则 -1
}

/* ==================== x,y,z 协同指令解析（Task1.3） ====================
 * 功能：解析 "x角度,y角度,z角度" 并同步控制三个舵机
 * 映射：x → 底座(D9)、y → 后臂(D8)、z → 前臂(D7)，夹爪保持不动
 * 说明：字母大小写均可（统一转小写后解析） */
void parseXYZ(const char *line){
  char buf[32];                                     // 局部缓冲（可修改）
  strncpy(buf, line, sizeof(buf) - 1);              // 复制指令
  buf[sizeof(buf) - 1] = '\0';                      // 保证字符串结束
  for (int i = 0; buf[i]; i++) buf[i] = tolower(buf[i]); // 统一小写

  int xi = -1, yi = -1, zi = -1;                    // 三个字母的位置（-1=未找到）
  for (int i = 0; buf[i]; i++){                     // 扫描定位
    if (buf[i] == 'x' && xi < 0) xi = i;
    else if (buf[i] == 'y' && yi < 0) yi = i;
    else if (buf[i] == 'z' && zi < 0) zi = i;
  }
  if (xi < 0 || yi < 0 || zi < 0){                  // 缺少任一字母
    Serial.println("! 格式错误：应为 x角度,y角度,z角度，例如 x10,y30,z20");
    return;
  }
  int vx = getNumberAfter(buf, xi + 1);             // 解析 x 后的角度
  int vy = getNumberAfter(buf, yi + 1);             // 解析 y 后的角度
  int vz = getNumberAfter(buf, zi + 1);             // 解析 z 后的角度
  if (vx < 0 || vy < 0 || vz < 0){                  // 任一角度缺失
    Serial.println("! 角度解析失败，请检查格式");
    return;
  }
  Serial.print("+ 协同控制：x底座="); Serial.print(vx);
  Serial.print(" y后臂="); Serial.print(vy);
  Serial.print(" z前臂="); Serial.println(vz);
  syncMove(vx, vy, vz, curClaw, servoDelay);       // 同步移动三舵机，夹爪不动
}

/* ==================== 串口指令解析（总入口） ====================
 * 功能：按指令格式分发到三个任务的对应功能
 * 规则：
 *   含逗号        → x,y,z 协同指令（Task1.3）
 *   单字符        → O/S/H/L/C/?（Task1.2 固定指令 + 辅助）
 *   首字母+数字   → b/r/f/c 单关节移动（Task1.1）
 *   其余          → 未知指令提示 */
void handleCommand(const char *line){
  int len = strlen(line);                           // 指令长度
  if (len == 0) return;                             // 空指令忽略

  if (strchr(line, ',') != NULL){                   // 含逗号 → 协同指令
    parseXYZ(line);
    return;
  }

  char c = tolower(line[0]);                        // 取首字符转小写
  if (len == 1){                                    // 单字符指令
    switch (c){
      case 'o': clawOpen();  break;                 // O：夹爪张开
      case 's': clawClose(); break;                 // S：夹爪闭合
      case 'h': speedUp();   break;                 // H：提升速度
      case 'l': speedDown(); break;                 // L：降低速度
      case 'c': centerAll(); break;                 // C：回中
      case '?': reportStatus(); break;              // ?：状态查询
      default:  Serial.println("! 未知指令（O/S/H/L/C/? 或 b角度/r角度/f角度/c角度 / x,y,z）");
    }
    return;
  }

  if ((c == 'b' || c == 'r' || c == 'f' || c == 'c') && isDigit(line[1])){ // b90 形式
    int v = getNumberAfter(line, 1);                // 解析角度数值
    if (v < 0){ Serial.println("! 缺少角度数值"); return; }
    switch (c){                                     // 按关节分发
      case 'b': moveBase(v); break;                 // 底座
      case 'r': moveRArm(v); break;                 // 后臂
      case 'f': moveFArm(v); break;                 // 前臂
      case 'c': moveClaw(v); break;                 // 夹爪
    }
    return;
  }
  Serial.println("! 未知指令");                     // 其它内容一律提示
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
  Serial.println("=== Task1 整合版：基础控制 + 固定指令通信 + 多舵机协同 ===");
  Serial.println("b角度/r角度/f角度/c角度 单关节 | O张开 S闭合 | H/L 速度 | C回中 ?状态 | x,y,z 协同");
}

/* ==================== 主循环 ====================
 * 每轮做两件事：
 *   1. 处理串口指令（非阻塞）
 *   2. 摇杆实时操控（速度控制，摇杆回中则不动） */
void loop(){
  readSerialLine();                                 // 处理串口指令

  /* 摇杆通道映射（方向系数已在配置区定义） */
  int dRArm = DIR_JOY1_X * axisDelta(JOY1_X);      // 后臂增量
  int dBase = DIR_JOY1_Y * axisDelta(JOY1_Y);      // 底座增量
  int dFArm = DIR_JOY2_X * axisDelta(JOY2_X);      // 前臂增量
  int dClaw = DIR_JOY2_Y * axisDelta(JOY2_Y);      // 夹爪增量
  if (dBase || dRArm || dFArm || dClaw){            // 任一摇杆轴有输入
    if (dBase){ curBase = clampVal(curBase + dBase, BASE_MIN, BASE_MAX); base.write(curBase); } // 底座
    if (dRArm){ curRArm = clampVal(curRArm + dRArm, RARM_MIN, RARM_MAX); rArm.write(curRArm); } // 后臂
    if (dFArm){ curFArm = clampVal(curFArm + dFArm, FARM_MIN, FARM_MAX); fArm.write(curFArm); } // 前臂
    if (dClaw){ curClaw = clampVal(curClaw + dClaw, CLAW_MIN, CLAW_MAX); claw.write(curClaw); } // 夹爪
    delay(servoDelay);                              // 每步延时 = 当前速度档
  }
}
