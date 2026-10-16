/*
 * ============================================================================
 *  文件：Task3.ino
 *  工程：Task3 —— 按键控制与录制播放（考核方向内 20%，联合电路方向）
 * ----------------------------------------------------------------------------
 *  考核点：电路方向自制 Arduino 板捕获 4 个按键 → 通过串口发送指令 →
 *          本 Arduino 接收后驱动机械臂执行：
 *
 *    按键1（循环执行，10%）：循环触发任务二的夹取动作 A→B→C→A→…
 *    按键2（录制）：         按一次开始录制（摇杆操控机械臂，时长>10s、
 *                           体现明显位移），再按一次结束并保存动作数据
 *    按键3（播放，10%）：    自动调取上一次录制数据并完整复现
 *    按键4（回中，5%）：     回归初始姿态
 *
 *  串口协议（自制板 → 本板，9600 波特率）：
 *    '1' 循环夹取    '2' 录制开始/结束    '3' 播放    '4' 回中
 *   （同时保留 D2/D3 摇杆按键作为本地调试备用：左摇杆=播放, 右摇杆=回中）
 *
 *  录制设计：
 *    每 REC_SAMPLE_MS 采样一帧（4 关节角度），最多 REC_MAX_FRAMES 帧
 *    默认 50ms × 220帧 ≈ 11 秒，满足考核">10秒"要求
 *    内存估算：220 帧 × 4 字节 ≈ 880B，Arduino Uno 2KB SRAM 内安全
 *
 *  硬件接线：
 *    舵机：base=D9  rArm=D8  fArm=D7  claw=D6
 *    摇杆：A3/A2/A1/A0（录制时操控用）
 *    按键：D2/D3（本地备用）；LED D13（录制指示）
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库（标准库）

/* ---------------- 引脚定义 ----------------
 * 舵机：D9/D8/D7/D6（不可改动，与机械臂接线一致）
 * 按键：由第二块板（KeypadBoard.ino）读取，通过串口 '1'~'4' 发来 */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6
#define PIN_SW1   2           // 左摇杆按键（本地备用：播放，ArmJoystick V3.0 固定 D2）
#define PIN_SW2   3           // 右摇杆按键（本地备用：回中，ArmJoystick V3.0 固定 D3）
#define PIN_LED   13          // 板载 LED（录制状态指示）

/* ---------------- 摇杆模拟引脚 ----------------
 * 录制时用摇杆操控机械臂，映射与 Task1.1/test1.1 一致 */
#define JOY1_X    A3          // 右摇杆上下 → 后臂
#define JOY1_Y    A2          // 右摇杆左右 → 底座
#define JOY2_X    A1          // 左摇杆上下 → 前臂
#define JOY2_Y    A0          // 左摇杆左右 → 夹爪

/* ---------------- 舵机限位 ---------------- */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  45          // 后臂角度下限（对齐 JoyFK_alpha2）
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  110         // 前臂角度上限（对齐 JoyFK_alpha2）
#define CLAW_MIN  5           // 夹爪张开极限
#define CLAW_MAX  130         // 夹爪闭合极限（对齐 JoyFK_alpha2）

/* ---------------- 摇杆参数 ---------------- */
#define JOY_CENTER   512      // 摇杆模拟值中点
#define JOY_DEADBAND 25       // 死区（中位 ±25 内不动）
#define JOY_MAX_STEP 1        // 单轮最大角度增量

/* ---------------- 摇杆方向配置（初始化时校准） ----------------
 * 与 Task1.1/test1.1 实测一致：+1=正向，-1=反向。
 *   右摇杆上下=A3 → 后臂(-1)    右摇杆左右=A2 → 底座(-1)
 *   左摇杆上下=A1 → 前臂(-1)    左摇杆左右=A0 → 夹爪(+1)
 * 录制时摇杆操控方向与 Task1.1 完全一致 */
#define DIR_JOY1_X  -1         // 后臂方向系数
#define DIR_JOY1_Y  -1         // 底座方向系数
#define DIR_JOY2_X  -1         // 前臂方向系数
#define DIR_JOY2_Y  +1         // 夹爪方向系数

/* ---------------- 录制参数 ---------------- */
#define REC_SAMPLE_MS   50    // 采样间隔 50ms → 220帧≈11秒，满足 >10s 要求
#define REC_MAX_FRAMES  220   // 最多帧数（4字节/帧，共约 880B RAM）

/* ---------------- 运动速度 ---------------- */
int servoDelay = 15;          // 每转 1° 的延时毫秒数

/* ---------------- 舵机对象与当前角度 ---------------- */
Servo base, rArm, fArm, claw; // 4 个舵机对象
int curBase = 90;             // 底座当前角度
int curRArm = 90;             // 后臂当前角度
int curFArm = 90;             // 前臂当前角度
int curClaw = 90;             // 夹爪当前角度

/* ==================== 录制帧缓冲 ====================
 * 每帧保存 4 个关节角度（uint8_t 足够，角度 0~180 < 256）
 * 录制时按时间间隔逐帧记录，播放时逐帧还原 */
struct Frame { uint8_t b, r, f, c; };  // 一帧 = 底座/后臂/前臂/夹爪角度
Frame frames[REC_MAX_FRAMES];          // 帧缓冲数组
int   recCount = 0;                    // 已录制的帧数
bool  recording = false;               // 是否正在录制
bool  hasRecord = false;               // 是否已有可播放的数据

/* ---------------- 串口行缓冲 ---------------- */
char lineBuf[16];             // 一行指令的字符缓冲
int  lineLen = 0;             // 缓冲中已接收的字符数

/* ==================== 点位定义（按实物标定！） ====================
 * 与 Task2 相同：按键1 循环夹取 A/B/C 时使用这些点位。
 * 目前为示例值，联调时须按实物标定后填入。 */
struct Pose { int b, r, f, c; };
const Pose HOME    = { 85, 30, 45, 60 };   // 待机位（与 Task2 同步）
const Pose GRAB_A  = { 46, 133, 30, 45 };  // 抓取 A（与 Task2 同步）
const Pose GRAB_B  = { 130, 107, 58, 45 }; // 抓取 B（与 Task2 同步）
const Pose GRAB_C  = { 107, 124, 52, 45 };  // 抓取 C（与 Task2 同步）
const Pose PLACE_A = { 126, 144, 48, 45 }; // 放置 A（与 Task2 同步）
const Pose PLACE_B = { 73, 155, 130, 45 };  // 放置 B（与 Task2 同步）
const Pose PLACE_C = { 32, 128, 31, 45 };   // 放置 C（与 Task2 同步）
const int GRIP_CLOSE = CLAW_MAX;           // 闭合=夹住
const int GRIP_OPEN  = CLAW_MIN;           // 张开=放下

/* ==================== 基础工具函数 ==================== */

/* 角度限位钳制：把 v 限制在 [lo, hi] 内 */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 多舵机同步移动 ====================
 * 功能：4 个关节同步逐度运动到目标角度（带限位保护） */
void syncMove(int tBase, int tRArm, int tFArm, int tClaw, int stepMs){
  tBase = clampVal(tBase, BASE_MIN, BASE_MAX);
  tRArm = clampVal(tRArm, RARM_MIN, RARM_MAX);
  tFArm = clampVal(tFArm, FARM_MIN, FARM_MAX);
  tClaw = clampVal(tClaw, CLAW_MIN, CLAW_MAX);
  bool done = false;
  while (!done){
    done = true;
    if (curBase != tBase){ curBase += (tBase > curBase) ? 1 : -1; base.write(curBase);  done = false; }
    if (curRArm != tRArm){ curRArm += (tRArm > curRArm) ? 1 : -1; rArm.write(curRArm); done = false; }
    if (curFArm != tFArm){ curFArm += (tFArm > curFArm) ? 1 : -1; fArm.write(curFArm); done = false; }
    if (curClaw != tClaw){ curClaw += (tClaw > curClaw) ? 1 : -1; claw.write(curClaw); done = false; }
    delay(stepMs);
  }
}

/* 整体移动到某点位 */
void moveTo(const Pose &p, int stepMs){ syncMove(p.b, p.r, p.f, p.c, stepMs); }

/* ==================== 夹取并放置（供按键1使用） ====================
 * 完整动作序列：回待机→下降抓取→闭合→抬升→平移→下降→张开→回待机 */
void pickAndPlace(const Pose &grab, const Pose &place, int lift = 15){
  moveTo(HOME, servoDelay);                         // 回待机
  int hiR = grab.r;                                 // 高度位后臂角
  int hiF = grab.f - lift;                          // 高度位前臂角（上抬）
  syncMove(grab.b, hiR, hiF, GRIP_OPEN, servoDelay);// 到抓取点上方
  syncMove(grab.b, grab.r, grab.f, GRIP_OPEN, servoDelay); // 下降
  delay(300);
  syncMove(grab.b, grab.r, grab.f, GRIP_CLOSE, servoDelay); // 闭合夹爪
  delay(300);
  syncMove(grab.b, hiR, hiF, GRIP_CLOSE, servoDelay);// 抬升
  syncMove(place.b, hiR, hiF, GRIP_CLOSE, servoDelay); // 平移
  syncMove(place.b, place.r, place.f, GRIP_CLOSE, servoDelay); // 下降
  delay(300);
  syncMove(place.b, place.r, place.f, GRIP_OPEN, servoDelay); // 张开放下
  delay(300);
  moveTo(HOME, servoDelay);                         // 回待机
}

/* ==================== 回中 ====================
 * 功能：所有关节回到初始姿态（90/90/90/夹爪30） */
void centerAll(){
  Serial.println(">>> 回中");
  syncMove(90, 90, 90, 30, servoDelay);
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

/* ==================== 按键1：循环夹取 ====================
 * 功能：依次执行 夹取A→夹取B→夹取C→夹取A→… 循环
 * 实现：pickCycle 记录当前轮到哪个物体，执行后 +1 取模循环 */
int pickCycle = 0;            // 0→A 1→B 2→C 循环计数

void button1LoopPick(){
  Serial.print(">>> 循环夹取 第"); Serial.print(pickCycle + 1); Serial.println(" 个物体");
  switch (pickCycle){
    case 0: pickAndPlace(GRAB_A, PLACE_A); break;   // 第一次按：夹取A
    case 1: pickAndPlace(GRAB_B, PLACE_B); break;   // 第二次按：夹取B
    default: pickAndPlace(GRAB_C, PLACE_C); break;  // 第三次按：夹取C
  }
  pickCycle = (pickCycle + 1) % 3;                  // 循环：4,5,6... 回到A
}

/* ==================== 按键2：录制 ====================
 * 功能：切换录制状态（开始/结束）
 * 实现：
 *   首次按 → 清空缓冲、置 recording=true、LED 常亮提示录制中
 *   再次按 → 结束录制，记录帧数，LED 熄灭
 * 说明：录制期间需用摇杆操控机械臂完成一段 >10s 的明显位移动作 */
void button2Record(){
  if (!recording){                                  // 当前未在录制 → 开始
    recCount = 0;                                   // 清空旧数据
    recording = true;                               // 进入录制状态
    hasRecord = false;                              // 旧播放数据作废
    digitalWrite(PIN_LED, HIGH);                    // LED 常亮 = 录制中
    Serial.println(">>> 录制开始：请摇杆操控机械臂（>10秒明显位移），再按一次结束");
  } else {                                          // 当前录制中 → 结束
    recording = false;                              // 退出录制状态
    hasRecord = (recCount > 0);                     // 有帧才算有数据
    digitalWrite(PIN_LED, LOW);                     // LED 熄灭
    Serial.print(">>> 录制结束，共 "); Serial.print(recCount);
    Serial.println(" 帧");
  }
}

/* ==================== 录制采样 ====================
 * 功能：把当前 4 关节角度存入下一帧（由 loop 按 REC_SAMPLE_MS 定时调用）
 * 说明：帧数满时自动结束录制（防止缓冲溢出） */
void captureFrame(){
  if (recCount < REC_MAX_FRAMES){
    frames[recCount].b = curBase;                   // 存底座角度
    frames[recCount].r = curRArm;                   // 存后臂角度
    frames[recCount].f = curFArm;                   // 存前臂角度
    frames[recCount].c = curClaw;                   // 存夹爪角度
    recCount++;                                     // 帧数 +1
  }
}

/* ==================== 按键3：播放 ====================
 * 功能：逐帧复现上一次录制动作
 * 实现：对每帧调用 syncMove 平滑移动到该帧角度（每帧延时 5ms/度）
 * 说明：若没有录制数据则提示并返回 */
void button3Play(){
  if (!hasRecord){                                  // 无数据
    Serial.println("! 没有可播放的录制数据");
    return;
  }
  Serial.print(">>> 播放 "); Serial.print(recCount); Serial.println(" 帧");
  for (int i = 0; i < recCount; i++){               // 逐帧还原
    syncMove(frames[i].b, frames[i].r, frames[i].f, frames[i].c, 5);
    delay(1);                                       // 帧间小延时保证稳定
  }
  Serial.println(">>> 播放完成");
}

/* ==================== 串口指令解析 ====================
 * 功能：处理一行串口指令（自制板按键指令 '1'~'4'） */
void handleCommand(const char *line){
  int len = strlen(line);
  if (len == 0) return;
  char c = line[0];                                 // 取首字符
  switch (c){                                       // 按指令分发
    case '1': button1LoopPick(); break;             // 按键1：循环夹取
    case '2': button2Record(); break;               // 按键2：录制/结束
    case '3': button3Play(); break;                 // 按键3：播放
    case '4': centerAll(); break;                   // 按键4：回中
    default:  Serial.println("! 未知指令（1循环夹取/2录制/3播放/4回中）"); break;
  }
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

/* ==================== 本地按键检测 ====================
 * 功能：读取板载摇杆按键 D2/D3（本地调试备用，防抖 300ms）
 *  D2=左摇杆按键=播放    D3=右摇杆按键=回中
 * 说明：正式演示时按键来自电路方向自制板（走串口指令），
 *       这两个本地键仅方便独立调试 */
void checkLocalButtons(){
  static unsigned long lastSw1 = 0, lastSw2 = 0;    // 上次触发时间（防抖）
  unsigned long now = millis();
  if (digitalRead(PIN_SW1) == LOW && now - lastSw1 > 300){ lastSw1 = now; button3Play(); } // 左摇杆按键=播放
  if (digitalRead(PIN_SW2) == LOW && now - lastSw2 > 300){ lastSw2 = now; centerAll(); }   // 右摇杆按键=回中
}

/* ==================== 初始化 ==================== */
void setup(){
  pinMode(PIN_SW1, INPUT_PULLUP);                   // D2 输入上拉（按下=低电平）
  pinMode(PIN_SW2, INPUT_PULLUP);                   // D3 输入上拉
  pinMode(PIN_LED, OUTPUT);                         // LED 输出
  digitalWrite(PIN_LED, LOW);                       // 初始熄灭

  base.attach(PIN_BASE);  delay(200);
  rArm.attach(PIN_RARM);  delay(200);
  fArm.attach(PIN_FARM);  delay(200);
  claw.attach(PIN_CLAW);  delay(200);

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90);
  delay(300);

  Serial.begin(9600);
  Serial.println("=== Task3 按键控制与录制播放 ===");
  Serial.println("串口指令：1循环夹取 | 2录制 | 3播放 | 4回中");
  Serial.println("（录制时请摇杆操控机械臂）");
}

/* ==================== 主循环 ====================
 * 每轮：
 *   1. 处理串口指令
 *   2. 检测本地按键（D4/D5）
 *   3. 若处于录制模式：摇杆实时操控 + 定时采样记录 */
void loop(){
  readSerialLine();                                 // 串口指令
  checkLocalButtons();                              // 本地按键

  if (recording){                                   // 录制模式
    /* 摇杆实时操控（方向系数见配置区 DIR_JOYx 常量，与 Task1.1 一致） */
    int dRArm = DIR_JOY1_X * axisDelta(JOY1_X);     // 右摇杆上下 → 后臂
    int dBase = DIR_JOY1_Y * axisDelta(JOY1_Y);     // 右摇杆左右 → 底座
    int dFArm = DIR_JOY2_X * axisDelta(JOY2_X);     // 左摇杆上下 → 前臂
    int dClaw = DIR_JOY2_Y * axisDelta(JOY2_Y);     // 左摇杆左右 → 夹爪
    if (dBase || dRArm || dFArm || dClaw){          // 有摇杆输入
      if (dBase){ curBase = clampVal(curBase + dBase, BASE_MIN, BASE_MAX); base.write(curBase); }
      if (dRArm){ curRArm = clampVal(curRArm + dRArm, RARM_MIN, RARM_MAX); rArm.write(curRArm); }
      if (dFArm){ curFArm = clampVal(curFArm + dFArm, FARM_MIN, FARM_MAX); fArm.write(curFArm); }
      if (dClaw){ curClaw = clampVal(curClaw + dClaw, CLAW_MIN, CLAW_MAX); claw.write(curClaw); }
      delay(10);                                    // 操控节拍
    }
    /* 定时采样：每 REC_SAMPLE_MS 记录一帧 */
    static unsigned long lastSample = 0;            // 上次采样时间
    if (millis() - lastSample >= REC_SAMPLE_MS){
      lastSample = millis();
      captureFrame();                               // 记录当前帧
      if (recCount >= REC_MAX_FRAMES){              // 帧数已满
        recording = false;                          // 自动结束录制
        hasRecord = true;
        digitalWrite(PIN_LED, LOW);
        Serial.println("! 帧数已满，自动结束录制");
      }
    }
  }
}
