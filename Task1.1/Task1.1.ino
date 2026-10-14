/*
 * ============================================================================
 *  文件：Task1.1.ino
 *  工程：Task1.1 —— 机械臂基本功能 · 基础控制（考核方向内 5%）
 * ----------------------------------------------------------------------------
 *  考核点：
 *    ① 各关节的独立移动（串口 b/r/f/c 单关节指令）
 *    ② 摇杆对机械臂的实时操控（速度控制 + 死区）
 *
 *  功能概述：
 *    1. 摇杆实时操控 4 个关节：推杆越远关节转动越快，摇杆回中则不动
 *       （速度控制模式，避免误触，保护舵机）
 *    2. 串口单关节独立移动：移动指定关节时，其余关节保持不动
 *    3. C 指令一键回中，? 指令查看当前各关节角度
 *
 *  摇杆通道映射（按 test1.1 实测校准，方向取反同 test1.1）：
 *        右摇杆 上下=A3 → 后臂(D8)   （方向取反：上推抬臂、下拉放臂）
 *        右摇杆 左右=A2 → 底座(D9)   （方向取反）
 *        左摇杆 上下=A1 → 前臂(D7)   （方向取反）
 *        左摇杆 左右=A0 → 夹爪(D6)   （不取反）
 *
 *  串口协议（9600 波特率，指令以回车/换行结束）：
 *        b角度  底座转到指定角度（如 b90）
 *        r角度  后臂转到指定角度（如 r60）
 *        f角度  前臂转到指定角度（如 f45）
 *        c角度  夹爪转到指定角度（如 c30）
 *        C      所有关节回到初始姿态（90/90/90/30）
 *        ?      打印当前各关节角度
 *
 *  硬件接线：
 *        舵机：base=D9  rArm=D8  fArm=D7  claw=D6（信号线）
 *        摇杆：A3/A2/A1/A0（模拟输入）
 *        本任务未使用按键 D4/D5
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库（标准库）

/* ---------------- 舵机引脚定义 ----------------
 * 每个舵机占用一个数字引脚，通过 PWM 信号控制角度。
 * 引脚命名与机械臂关节对应：base=底座、rArm=后臂(大臂)、
 * fArm=前臂(小臂)、claw=夹爪。 */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6

/* ---------------- 摇杆模拟引脚定义 ----------------
 * 双摇杆扩展板上每个摇杆有 X/Y 两个模拟轴，读值为 0~1023，中点约 512。
 * 具体通道与关节的对应关系见文件头注释。 */
#define JOY1_X    A3          // 右摇杆上下轴 → 后臂
#define JOY1_Y    A2          // 右摇杆左右轴 → 底座
#define JOY2_X    A1          // 左摇杆上下轴 → 前臂
#define JOY2_Y    A0          // 左摇杆左右轴 → 夹爪

/* ---------------- 舵机限位（超限自动限制） ----------------
 * 每个关节有机械行程限制，超出后舵机会堵转（咔咔响、发烫、损坏）。
 * 程序内强制钳制角度到安全范围；后臂上限 150 是实测安全值，
 * 夹爪 5~100 为实测限位（若夹爪方向相反可交换 CLAW_MIN/CLAW_MAX）。 */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  30          // 后臂角度下限
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  130         // 前臂角度上限
#define CLAW_MIN  5           // 夹爪张开极限角度
#define CLAW_MAX  100         // 夹爪闭合极限角度

/* ---------------- 摇杆控制参数 ----------------
 * 摇杆是"速度控制"而非"位置跟随"：
 * 只有当摇杆偏离中位超过死区时，关节才朝对应方向转动，
 * 偏离越远转动越快（单轮最大增量 JOY_MAX_STEP 度）。 */
#define JOY_CENTER   512      // 摇杆模拟值中点（未拨动时的读值）
#define JOY_DEADBAND 25       // 死区：中位 ±25 内视为"未拨动"，防止误触抖动
#define JOY_MAX_STEP 1        // 单轮循环最大角度增量（度）；值越大越灵敏

/* ---------------- 摇杆方向配置（初始化时校准） ----------------
 * 摇杆推杆方向由硬件固定（上推→模拟值增大），而关节正方向由舵机
 * 安装决定；按 test1.1 实测，以下 3 个轴推杆方向与期望运动相反，
 * 故方向系数为 -1（取反），夹爪恰好一致为 +1。
 * 改接线/换舵机后若某轴方向反了，只需调整对应常量，不动运行逻辑。
 *   右摇杆上下=A3 → 后臂(-1)    右摇杆左右=A2 → 底座(-1)
 *   左摇杆上下=A1 → 前臂(-1)    左摇杆左右=A0 → 夹爪(+1) */
#define DIR_JOY1_X  -1         // 后臂方向系数
#define DIR_JOY1_Y  -1         // 底座方向系数
#define DIR_JOY2_X  -1         // 前臂方向系数
#define DIR_JOY2_Y  +1         // 夹爪方向系数

/* ---------------- 运动速度参数 ----------------
 * servoDelay 是"每转动 1° 的延时毫秒数"：
 *   延时越大 → 舵机转得越慢；延时越小 → 转得越快。
 * 同时用于同步移动函数，保证 4 个关节按统一速度运动。 */
int servoDelay = 15;          // 默认每度延时 15ms（速度适中）

/* ---------------- 舵机对象与当前角度 ----------------
 * 全局记录"当前实际角度"，所有运动都基于当前值做逐度逼近，
 * 这样串口指令和摇杆操作可以无缝叠加，不会出现角度跳变。 */
Servo base, rArm, fArm, claw; // 4 个舵机对象（attach 后可用 .write 控制）
int curBase = 90;             // 底座当前角度（初始化 90）
int curRArm = 90;             // 后臂当前角度
int curFArm = 90;             // 前臂当前角度
int curClaw = 90;             // 夹爪当前角度

/* ---------------- 串口行缓冲 ----------------
 * 逐字符接收串口数据，遇到回车/换行才作为一条完整指令处理，
 * 支持一次输入多条指令（程序非阻塞，不丢指令）。 */
char lineBuf[32];             // 一行指令的字符缓冲
int  lineLen = 0;             // 缓冲中已接收的字符数

/* ==================== 角度限位钳制 ====================
 * 功能：把角度 v 限制在 [lo, hi] 范围内
 * 参数：v 待限制的角度；lo/hi 允许的最小/最大角度
 * 返回：限制后的安全角度
 * 说明：超出范围的角度会被"截断"到边界，而不是取模或丢弃 */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 摇杆轴增量读取 ====================
 * 功能：读取某摇杆轴当前偏离中位的程度，换算成角度增量
 * 参数：pin 摇杆轴对应的模拟引脚（A0~A3）
 * 返回：死区内返回 0；向一端推返回 +1~+JOY_MAX_STEP，
 *       向另一端推返回 -1~-JOY_MAX_STEP（符号代表转动方向）
 * 说明：实现"速度控制"手感的关键函数——
 *       中位 ±25 内输出 0（不动），越往外推输出越大（转越快） */
int axisDelta(int pin){
  int v = analogRead(pin);                          // 读取模拟值 0~1023
  if (v > JOY_CENTER + JOY_DEADBAND){               // 正向推过了死区
    int span = 1023 - (JOY_CENTER + JOY_DEADBAND);  // 可用的正向推杆行程
    if (span <= 0) return 1;                        // 防御：行程异常时最小步进
    return map(v - (JOY_CENTER + JOY_DEADBAND), 0, span, 1, JOY_MAX_STEP); // 按偏移比例映射增量
  }
  if (v < JOY_CENTER - JOY_DEADBAND){               // 反向推过了死区
    int span = JOY_CENTER - JOY_DEADBAND;           // 可用的反向推杆行程
    if (span <= 0) return -1;                       // 防御：行程异常时最小步进
    return -map((JOY_CENTER - JOY_DEADBAND) - v, 0, span, 1, JOY_MAX_STEP); // 负号=反向转动
  }
  return 0;                                         // 在死区内：不动
}

/* ==================== 多舵机同步移动 ====================
 * 功能：让 4 个关节"同时、逐度"运动到目标角度
 * 参数：tBase/tRArm/tFArm/tClaw 各自的目标角度；stepMs 每步延时（速度）
 * 说明：这是全程序最核心的运动函数——
 *   ① 先对目标角度做限位钳制（防超行程损坏舵机）
 *   ② while 循环每轮只让每个关节走 1°，直到全部到位
 *   ③ 因为是每轮所有关节各走 1°，所以 4 个关节"同步开始、同步到位"
 *      （单关节独立移动 = 只传一个目标，其余传当前值，自然保持不动） */
void syncMove(int tBase, int tRArm, int tFArm, int tClaw, int stepMs){
  tBase = clampVal(tBase, BASE_MIN, BASE_MAX);      // 底座角度限位
  tRArm = clampVal(tRArm, RARM_MIN, RARM_MAX);      // 后臂角度限位
  tFArm = clampVal(tFArm, FARM_MIN, FARM_MAX);      // 前臂角度限位
  tClaw = clampVal(tClaw, CLAW_MIN, CLAW_MAX);      // 夹爪角度限位

  bool done = false;                                // 是否所有关节都已到位
  while (!done){
    done = true;                                    // 先假定全部到位，下面检查
    /* 每个关节：若未到目标，朝目标方向走 1° 并写入舵机 */
    if (curBase != tBase){ curBase += (tBase > curBase) ? 1 : -1; base.write(curBase);  done = false; }
    if (curRArm != tRArm){ curRArm += (tRArm > curRArm) ? 1 : -1; rArm.write(curRArm); done = false; }
    if (curFArm != tFArm){ curFArm += (tFArm > curFArm) ? 1 : -1; fArm.write(curFArm); done = false; }
    if (curClaw != tClaw){ curClaw += (tClaw > curClaw) ? 1 : -1; claw.write(curClaw); done = false; }
    delay(stepMs);                                  // 每步延时，控制整体速度
  }
}

/* ==================== 单关节独立移动 ====================
 * 功能：只移动指定关节到目标角度，其余关节保持当前位置
 * 实现：把目标关节传目标角度，其余 3 个传"当前角度"→ 它们自然不动
 *       （syncMove 内部判断"当前==目标"就不会走） */
void moveBase (int t){ syncMove(t,          curRArm, curFArm, curClaw, servoDelay); } // 底座
void moveRArm (int t){ syncMove(curBase, t,          curFArm, curClaw, servoDelay); } // 后臂
void moveFArm (int t){ syncMove(curBase, curRArm, t,          curClaw, servoDelay); } // 前臂
void moveClaw (int t){ syncMove(curBase, curRArm, curFArm, t,          servoDelay); } // 夹爪

/* ==================== 回中 ====================
 * 功能：所有关节回到初始姿态（90/90/90/夹爪30）
 * 说明：夹爪回中 30°（半开状态），与 test1.1 保持一致 */
void centerAll(){
  Serial.println(">>> 回中");                       // 串口提示
  syncMove(90, 90, 90, 30, servoDelay);             // 同步回到初始姿态
}

/* ==================== 数字解析 ====================
 * 功能：从字符串 s 的 idx 位置开始解析一个十进制数字
 * 参数：s 指令字符串；idx 起始下标（例如 'b' 之后的下标）
 * 返回：解析到的整数；若该位置没有数字返回 -1
 * 说明：跳过前导空格，只认 0~9 字符，遇到非数字即停止 */
int getNumberAfter(const char *s, int idx){
  int v = 0;                                        // 累加结果
  bool any = false;                                 // 是否真的解析到了数字
  while (s[idx] == ' ') idx++;                      // 跳过前导空格
  while (s[idx] >= '0' && s[idx] <= '9'){           // 逐位累加数字
    v = v * 10 + (s[idx] - '0');
    any = true;
    idx++;
  }
  return any ? v : -1;                              // 有数字返回数值，否则 -1
}

/* ==================== 串口指令解析 ====================
 * 功能：处理一条完整指令（由 readSerialLine 整行传来）
 * 支持：b/r/f/c 角度 → 单关节独立移动；C → 回中；? → 状态
 * 说明：tolower 使单字母指令大小写不敏感；
 *       角度指令要求第 2 个字符是数字，避免误判 */
void handleCommand(const char *line){
  int len = strlen(line);                           // 指令长度
  if (len == 0) return;                             // 空指令忽略
  char c = tolower(line[0]);                        // 取首字符并转小写

  if (len == 1){                                    // 单字符指令（C/?）
    if (c == 'c') centerAll();                      // C：回中
    else if (line[0] == '?') reportStatus();        // ?：状态查询
    else Serial.println("! 未知指令（b角度/r角度/f角度/c角度 / C回中 / ?状态）");
    return;
  }
  if ((c == 'b' || c == 'r' || c == 'f' || c == 'c') && isDigit(line[1])){ // b90 形式
    int v = getNumberAfter(line, 1);                // 解析角度数值
    if (v < 0){ Serial.println("! 缺少角度数值"); return; } // 解析失败
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
 * 功能：非阻塞地从串口读取字符，拼成完整一行后调用 handleCommand
 * 说明：放在 loop 里每轮调用；遇到 \n 或 \r 表示一行结束；
 *       缓冲溢出时清空该行（防止脏数据撑爆缓冲） */
void readSerialLine(){
  while (Serial.available()){                       // 串口接收缓冲有数据
    char ch = Serial.read();                        // 读一个字符
    if (ch == '\n' || ch == '\r'){                  // 行结束符
      if (lineLen > 0){                             // 行非空才处理
        lineBuf[lineLen] = '\0';                    // 补字符串结束符
        handleCommand(lineBuf);                     // 解析并执行
        lineLen = 0;                                // 清空缓冲等下一行
      }
    } else if (lineLen < (int)sizeof(lineBuf) - 1){ // 缓冲还有空间
      lineBuf[lineLen++] = ch;                      // 存入缓冲
    } else {
      lineLen = 0;                                  // 溢出：丢弃本行数据
    }
  }
}

/* ==================== 状态报告 ====================
 * 功能：向串口打印 4 个关节的当前角度，便于联调观察 */
void reportStatus(){
  Serial.println("----- 机械臂状态 -----");
  Serial.print("底座="); Serial.print(curBase);     // 底座角度
  Serial.print(" 后臂="); Serial.print(curRArm);    // 后臂角度
  Serial.print(" 前臂="); Serial.print(curFArm);    // 前臂角度
  Serial.print(" 夹爪="); Serial.println(curClaw);  // 夹爪角度
  Serial.println("------------------------");
}

/* ==================== 初始化 ====================
 * 上电流程：
 *   1. 依次 attach 4 个舵机（每次延时 200ms 保证舵机初始化稳定）
 *   2. 全部写 90° 作为初始姿态
 *   3. 打开串口 9600，打印欢迎信息和指令说明 */
void setup(){
  base.attach(PIN_BASE);  delay(200);               // 底座舵机接上引脚 9
  rArm.attach(PIN_RARM);  delay(200);               // 后臂舵机接上引脚 8
  fArm.attach(PIN_FARM);  delay(200);               // 前臂舵机接上引脚 7
  claw.attach(PIN_CLAW);  delay(200);               // 夹爪舵机接上引脚 6

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90); // 初始 90°
  delay(300);                                       // 等舵机到位

  Serial.begin(9600);                               // 串口 9600
  Serial.println("=== Task1.1 基础控制：摇杆实时操控 + 单关节独立移动 ===");
  Serial.println("指令：b角度/r角度/f角度/c角度 单关节移动；C 回中；? 状态");
}

/* ==================== 主循环 ====================
 * 每轮做两件事：
 *   1. 检查串口指令（非阻塞）
 *   2. 读取 4 个摇杆轴，若有输入则按映射操控对应关节
 * 说明：摇杆是"速度控制"——摇杆回中时没有任何角度变化；
 *       摇杆朝某方向推时，对应关节每轮按 axisDelta 的增量转动 */
void loop(){
  readSerialLine();                                 // 处理串口指令

  /* 摇杆通道映射（方向系数已在配置区 DIR_JOYx 常量定义好）：
   *   右摇杆上下=A3 → 后臂 rArm    右摇杆左右=A2 → 底座 BASE
   *   左摇杆上下=A1 → 前臂 fArm    左摇杆左右=A0 → 夹爪 claw */
  int dRArm = DIR_JOY1_X * axisDelta(JOY1_X);      // 后臂增量
  int dBase = DIR_JOY1_Y * axisDelta(JOY1_Y);      // 底座增量
  int dFArm = DIR_JOY2_X * axisDelta(JOY2_X);      // 前臂增量
  int dClaw = DIR_JOY2_Y * axisDelta(JOY2_Y);      // 夹爪增量
  if (dBase || dRArm || dFArm || dClaw){            // 任一摇杆轴有输入
    if (dBase){ curBase = clampVal(curBase + dBase, BASE_MIN, BASE_MAX); base.write(curBase); } // 底座
    if (dRArm){ curRArm = clampVal(curRArm + dRArm, RARM_MIN, RARM_MAX); rArm.write(curRArm); } // 后臂
    if (dFArm){ curFArm = clampVal(curFArm + dFArm, FARM_MIN, FARM_MAX); fArm.write(curFArm); } // 前臂
    if (dClaw){ curClaw = clampVal(curClaw + dClaw, CLAW_MIN, CLAW_MAX); claw.write(curClaw); } // 夹爪
    delay(servoDelay);                              // 每步延时 = 摇杆操控速度
  }
}
