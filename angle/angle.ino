/*
 * ============================================================================
 *  文件：angle.ino
 *  工程：angle —— 摇杆控制 + 末端坐标实时显示 + XYZ 协同控制
 * ----------------------------------------------------------------------------
 *  功能说明：
 *    1. 摇杆实时操控 4 个关节（速度控制 + 死区，通道映射与 test1.1 一致）
 *    2. 串口每 300ms 自动刷新一行，显示当前坐标（0~180° 角度值）：
 *         X=xxx  Y=xxx  Z=xxx  C=xxx
 *         X=底座角度  Y=后臂角度  Z=前臂角度  C=夹爪角度
 *    3. 串口 XYZ 协同控制：一条指令同步控制三个臂关节
 *
 *  串口指令（9600 波特率，回车/换行结束，大小写均可）：
 *    x角度,y角度,z角度    同步控制 底座/后臂/前臂（例：x60,y35,z20）
 *    c角度                夹爪单独控制（例：c45）
 *    C                    所有关节回中（X=90 Y=90 Z=90 C=30）
 *    ?                    立即打印一次当前坐标
 *
 *  坐标口径：
 *    X、Y、Z 即三个臂关节的角度值，范围 0~180（与舵机角度一致）。
 *    映射：X→底座(9号引脚)  Y→后臂(8号引脚)  Z→前臂(7号引脚)，夹爪为 C。
 *
 *  硬件接线：
 *    舵机：base=D9  rArm=D8  fArm=D7  claw=D6
 *    摇杆：右摇杆上下=A3→后臂、右摇杆左右=A2→底座
 *          左摇杆上下=A1→前臂、左摇杆左右=A0→夹爪
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库

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
#define BASE_MIN  0           // 底座角度下限（X）
#define BASE_MAX  180         // 底座角度上限（X）
#define RARM_MIN  10          // 后臂角度下限（Y）
#define RARM_MAX  160         // 后臂角度上限（Y，防堵转）
#define FARM_MIN  30          // 前臂角度下限（Z）
#define FARM_MAX  130         // 前臂角度上限（Z）
#define CLAW_MIN  5           // 夹爪张开极限角度（C）
#define CLAW_MAX  100         // 夹爪闭合极限角度（C）

/* ---------------- 摇杆控制参数 ----------------
 * 摇杆是"速度控制"：推杆越远离中位，关节转动越快。 */
#define JOY_CENTER   512      // 摇杆模拟值中点
#define JOY_DEADBAND 25       // 死区：中位 ±25 内视为未拨动
#define JOY_MAX_STEP 1        // 单轮循环最大角度增量（度）

/* ---------------- 摇杆方向配置（按 test1.1 实测） ---------------- */
#define DIR_JOY1_X  -1        // 后臂方向系数
#define DIR_JOY1_Y  -1        // 底座方向系数
#define DIR_JOY2_X  -1        // 前臂方向系数
#define DIR_JOY2_Y  +1        // 夹爪方向系数

/* ---------------- 运动速度 ---------------- */
int servoDelay = 15;          // 每度延时毫秒数（越大越慢）

/* ---------------- 坐标显示参数 ---------------- */
#define PRINT_MS 300          // 串口自动刷新坐标的间隔毫秒数

/* ---------------- 舵机对象与当前坐标 ---------------- */
Servo base, rArm, fArm, claw; // 4 个舵机对象
int curX = 90;                // 当前 X = 底座角度
int curY = 90;                // 当前 Y = 后臂角度
int curZ = 90;                // 当前 Z = 前臂角度
int curC = 90;                // 当前 C = 夹爪角度

/* ---------------- 串口行缓冲 ---------------- */
char lineBuf[32];             // 一行指令缓冲
int  lineLen = 0;             // 已接收字符数
unsigned long lastPrint = 0;  // 上次坐标打印的时间戳

/* ==================== 角度限位钳制 ==================== */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 摇杆轴增量读取 ====================
 * 功能：读某摇杆轴偏离中位的程度，换算成角度增量
 * 返回：死区内 0；越往一端推 ±1~±JOY_MAX_STEP */
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

/* ==================== 多舵机同步移动 ====================
 * 功能：4 个关节同步逐度运动到目标坐标（带限位保护） */
void syncMove(int tX, int tY, int tZ, int tC, int stepMs){
  tX = clampVal(tX, BASE_MIN, BASE_MAX);
  tY = clampVal(tY, RARM_MIN, RARM_MAX);
  tZ = clampVal(tZ, FARM_MIN, FARM_MAX);
  tC = clampVal(tC, CLAW_MIN, CLAW_MAX);
  bool done = false;
  while (!done){
    done = true;
    if (curX != tX){ curX += (tX > curX) ? 1 : -1; base.write(curX); done = false; }
    if (curY != tY){ curY += (tY > curY) ? 1 : -1; rArm.write(curY); done = false; }
    if (curZ != tZ){ curZ += (tZ > curZ) ? 1 : -1; fArm.write(curZ); done = false; }
    if (curC != tC){ curC += (tC > curC) ? 1 : -1; claw.write(curC); done = false; }
    delay(stepMs);
  }
}

/* ==================== 回中 ====================
 * 功能：所有关节回到初始姿态（X=90 Y=90 Z=90 C=30） */
void centerAll(){
  syncMove(90, 90, 90, 30, servoDelay);
}

/* ==================== 打印一次当前坐标 ====================
 * 功能：输出当前坐标，格式与 XYZ 控制对应（0~180° 角度值）
 * 输出格式：X=xxx  Y=xxx  Z=xxx  C=xxx */
void printXYZ(){
  Serial.print("X=");  Serial.print(curX);
  Serial.print("  Y="); Serial.print(curY);
  Serial.print("  Z="); Serial.print(curZ);
  Serial.print("  C="); Serial.println(curC);
}

/* ==================== XYZ 协同指令解析 ====================
 * 功能：解析 "x角度,y角度,z角度" 并同步控制三个臂关节
 * 映射：X→底座、Y→后臂、Z→前臂，夹爪 C 保持不动
 * 例：x60,y35,z20 → 底座60°、后臂35°、前臂20° 同步运动 */
void parseXYZ(const char *line){
  char buf[32];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  for (int i = 0; buf[i]; i++) buf[i] = tolower(buf[i]); // 统一小写

  int xi = -1, yi = -1, zi = -1;                    // 三个字母的位置
  for (int i = 0; buf[i]; i++){
    if (buf[i] == 'x' && xi < 0) xi = i;
    else if (buf[i] == 'y' && yi < 0) yi = i;
    else if (buf[i] == 'z' && zi < 0) zi = i;
  }
  if (xi < 0 || yi < 0 || zi < 0){                  // 缺少任一字母
    Serial.println("! 格式错误：应为 x角度,y角度,z角度，例如 x60,y35,z20");
    return;
  }
  int vx = -1, vy = -1, vz = -1;
  for (int i = xi + 1; i < (int)strlen(buf) && buf[i] >= '0' && buf[i] <= '9'; i++)
    vx = (vx < 0 ? 0 : vx) * 10 + (buf[i] - '0');
  for (int i = yi + 1; i < (int)strlen(buf) && buf[i] >= '0' && buf[i] <= '9'; i++)
    vy = (vy < 0 ? 0 : vy) * 10 + (buf[i] - '0');
  for (int i = zi + 1; i < (int)strlen(buf) && buf[i] >= '0' && buf[i] <= '9'; i++)
    vz = (vz < 0 ? 0 : vz) * 10 + (buf[i] - '0');
  if (vx < 0 || vy < 0 || vz < 0){                  // 任一角度缺失
    Serial.println("! 角度解析失败，请检查格式");
    return;
  }
  Serial.print("+ 协同控制：X底座="); Serial.print(vx);
  Serial.print(" Y后臂="); Serial.print(vy);
  Serial.print(" Z前臂="); Serial.println(vz);
  syncMove(vx, vy, vz, curC, servoDelay);           // 同步移动三臂关节，夹爪不动
}

/* ==================== 串口指令解析 ====================
 * 功能：XYZ 协同控制 + 夹爪 c + C 回中 + ? 坐标查询 */
void handleCommand(const char *line){
  int len = strlen(line);
  if (len == 0) return;

  if (strchr(line, ',') != NULL){                   // 含逗号 → XYZ 协同指令
    parseXYZ(line);
    return;
  }

  char c = tolower(line[0]);
  if (len == 1){                                    // 单字符指令
    switch (c){
      case 'c': centerAll(); break;                 // C：回中
      case '?': printXYZ();  break;                 // ?：立即打印坐标
      default:  Serial.println("! 未知指令（x角度,y角度,z角度 / c角度 / C回中 / ?坐标）");
    }
    return;
  }

  if (c == 'c' && isDigit(line[1])){                // c角度：夹爪单独控制
    int v = 0, i = 1;
    while (line[i] >= '0' && line[i] <= '9'){ v = v * 10 + (line[i] - '0'); i++; }
    syncMove(curX, curY, curZ, v, servoDelay);      // 只动夹爪
    return;
  }
  Serial.println("! 未知指令");
}

/* ==================== 串口逐行读取 ==================== */
void readSerialLine(){
  while (Serial.available()){
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r'){
      if (lineLen > 0){
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < (int)sizeof(lineBuf) - 1){
      lineBuf[lineLen++] = ch;
    } else {
      lineLen = 0;
    }
  }
}

/* ==================== 初始化 ==================== */
void setup(){
  base.attach(PIN_BASE);  delay(200);               // 底座舵机接引脚 9
  rArm.attach(PIN_RARM);  delay(200);               // 后臂舵机接引脚 8
  fArm.attach(PIN_FARM);  delay(200);               // 前臂舵机接引脚 7
  claw.attach(PIN_CLAW);  delay(200);               // 夹爪舵机接引脚 6

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90);
  delay(300);                                       // 等舵机到位

  Serial.begin(9600);
  Serial.println("=== angle：摇杆控制 + 坐标显示 + XYZ 协同控制（0~180°）===");
  Serial.println("指令：x角度,y角度,z角度 同步控三臂 | c角度 夹爪 | C回中 | ?坐标");
}

/* ==================== 主循环 ====================
 * 每轮：
 *   1. 处理串口指令（XYZ 协同/夹爪/回中/查询）
 *   2. 摇杆实时操控（速度控制）
 *   3. 每 PRINT_MS 毫秒自动打印一次当前坐标 */
void loop(){
  readSerialLine();                                 // 处理串口指令

  /* 摇杆实时操控（通道映射见配置区） */
  int dY = DIR_JOY1_X * axisDelta(JOY1_X);          // 后臂增量（Y）
  int dX = DIR_JOY1_Y * axisDelta(JOY1_Y);          // 底座增量（X）
  int dZ = DIR_JOY2_X * axisDelta(JOY2_X);          // 前臂增量（Z）
  int dC = DIR_JOY2_Y * axisDelta(JOY2_Y);          // 夹爪增量（C）
  if (dX || dY || dZ || dC){                        // 任一摇杆轴有输入
    if (dX){ curX = clampVal(curX + dX, BASE_MIN, BASE_MAX); base.write(curX); }
    if (dY){ curY = clampVal(curY + dY, RARM_MIN, RARM_MAX); rArm.write(curY); }
    if (dZ){ curZ = clampVal(curZ + dZ, FARM_MIN, FARM_MAX); fArm.write(curZ); }
    if (dC){ curC = clampVal(curC + dC, CLAW_MIN, CLAW_MAX); claw.write(curC); }
    delay(servoDelay);                              // 每步延时 = 当前速度档
  }

  /* 定时刷新坐标显示 */
  if (millis() - lastPrint >= PRINT_MS){            // 到 300ms 就打印一次
    printXYZ();
    lastPrint = millis();
  }
}
