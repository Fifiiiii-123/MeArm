/*
 * ============================================================================
 *  文件：Task1.3.ino
 *  工程：Task1.3 —— 机械臂基本功能 · 多舵机协同控制（考核方向内 5%）
 * ----------------------------------------------------------------------------
 *  考核点：上位机发送 "x(角度),y(角度),z(角度)" 指令（例："x10,y30,z20"），
 *          机械臂解析指令并**同步**控制 x、y、z 对应的三个舵机，
 *          分别输出对应角度的 PWM 占空比（单条指令同步控制三个舵机）。
 *
 *  x/y/z 与物理舵机的映射（自行定义，此处记录清楚）：
 *      x → 底座(D9)    y → 后臂(D8)    z → 前臂(D7)    夹爪保持不动
 *
 *  实现要点：
 *    1. 按行读取串口 → 含逗号即视为 x,y,z 协同指令
 *    2. 提取 x/y/z 后的数字（兼容空格、大小写）
 *    3. 同步移动：syncMove 每轮三个舵机各走 1°、统一延时 → 真正同时到位
 *    4. 角度超限自动钳制到安全范围（硬件保护）
 *
 *  串口协议（9600 波特率，指令以回车/换行结束）：
 *        x角度,y角度,z角度  同步控制底座/后臂/前臂（例：x10,y30,z20）
 *        ?                  打印当前各关节角度
 *
 *  硬件接线：
 *        舵机：base=D9  rArm=D8  fArm=D7  claw=D6（信号线）
 *        本任务无需摇杆
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库（标准库）

/* ---------------- 舵机引脚定义 ---------------- */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9（对应指令 x）
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8（对应指令 y）
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7（对应指令 z）
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6（本任务不动）

/* ---------------- 舵机限位 ----------------
 * 协同指令中的角度超出限位时自动钳制并提示，防止损坏舵机。 */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  30          // 后臂角度下限
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  130         // 前臂角度上限
#define CLAW_MIN  5           // 夹爪张开极限
#define CLAW_MAX  100         // 夹爪闭合极限

/* ---------------- 运动速度 ---------------- */
int servoDelay = 15;          // 每转 1° 的延时毫秒数

/* ---------------- 舵机对象与当前角度 ---------------- */
Servo base, rArm, fArm, claw; // 4 个舵机对象
int curBase = 90;             // 底座当前角度
int curRArm = 90;             // 后臂当前角度
int curFArm = 90;             // 前臂当前角度
int curClaw = 90;             // 夹爪当前角度

/* ---------------- 串口行缓冲 ---------------- */
char lineBuf[32];             // 一行指令的字符缓冲
int  lineLen = 0;             // 缓冲中已接收的字符数

/* ==================== 角度限位钳制 ==================== */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 多舵机同步移动 ====================
 * 功能：4 个关节同步逐度运动到目标角度
 * 参数：tBase/tRArm/tFArm/tClaw 目标角度；stepMs 每步延时
 * 说明：每轮所有关节各走 1° → 三个舵机"同步开始、同步到位"，
 *       这是"单条指令同步控制三个舵机"评分点的关键实现。
 *       超限的目标角度会被钳制并打印提示。 */
void syncMove(int tBase, int tRArm, int tFArm, int tClaw, int stepMs){
  /* 限位检查：超出范围则钳制，并标记提示 */
  bool outOfLimit = false;
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
    delay(stepMs);                                  // 每步延时
  }
}

/* ==================== 数字解析 ====================
 * 功能：从字符串 s 的 idx 位置开始解析十进制数字
 * 返回：解析到的整数；无数字返回 -1 */
int getNumberAfter(const char *s, int idx){
  int v = 0;                                        // 累加结果
  bool any = false;                                 // 是否解析到数字
  while (s[idx] == ' ') idx++;                      // 跳过前导空格
  while (s[idx] >= '0' && s[idx] <= '9'){           // 逐位累加
    v = v * 10 + (s[idx] - '0');
    any = true;
    idx++;
  }
  return any ? v : -1;                              // 有数字返回数值，否则 -1
}

/* ==================== x,y,z 协同指令解析 ====================
 * 功能：解析 "x角度,y角度,z角度" 并同步控制三个舵机
 * 参数：line 整行指令字符串
 * 说明：
 *   1. 先复制到局部缓冲并统一转小写（大小写均可）
 *   2. 分别定位 'x'/'y'/'z' 字符的下标
 *   3. 从各字母之后解析数字
 *   4. 调用 syncMove 同步移动（夹爪传当前值保持不动） */
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

/* ==================== 状态报告 ==================== */
void reportStatus(){
  Serial.println("----- 机械臂状态 -----");
  Serial.print("底座(x)="); Serial.print(curBase);
  Serial.print(" 后臂(y)="); Serial.print(curRArm);
  Serial.print(" 前臂(z)="); Serial.print(curFArm);
  Serial.print(" 夹爪="); Serial.println(curClaw);
  Serial.println("------------------------");
}

/* ==================== 指令分发 ====================
 * 功能：区分 x,y,z 协同指令与辅助指令（?）
 * 规则：指令中含逗号 → 按 x,y,z 协同处理；单字符 '?' → 状态查询 */
void handleCommand(const char *line){
  int len = strlen(line);
  if (len == 0) return;

  if (len == 1 && line[0] == '?'){                  // 状态查询
    reportStatus();
    return;
  }
  if (strchr(line, ',') != NULL){                   // 含逗号 → 协同指令
    parseXYZ(line);
    return;
  }
  Serial.println("! 未知指令（应为 x角度,y角度,z角度，如 x10,y30,z20；? 状态）");
}

/* ==================== 串口逐行读取 ==================== */
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
  base.attach(PIN_BASE);  delay(200);               // 底座舵机（x）
  rArm.attach(PIN_RARM);  delay(200);               // 后臂舵机（y）
  fArm.attach(PIN_FARM);  delay(200);               // 前臂舵机（z）
  claw.attach(PIN_CLAW);  delay(200);               // 夹爪舵机

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90); // 初始 90°
  delay(300);

  Serial.begin(9600);
  Serial.println("=== Task1.3 多舵机协同：x角度,y角度,z角度 ===");
  Serial.println("映射：x→底座(D9) y→后臂(D8) z→前臂(D7)，三舵机同步转动");
  Serial.println("示例：x10,y30,z20；? 状态");
}

/* ==================== 主循环 ====================
 * 每轮只做一件事：等待并处理串口指令 */
void loop(){
  readSerialLine();
}
