 /*
 * ============================================================================
 *  文件：Task2.ino
 *  工程：Task2 —— 自动夹取与放置（考核方向内 15%）
 * ----------------------------------------------------------------------------
 *  考核点：上位机发送 "A"/"B"/"C"，机械臂自动完成夹取对应物体并放置到
 *          另一位置；三个放置位置须为三个明显不同的位置，且与物体初始
 *          位置在 x、y 方向上有明显位移。
 *
 *  物体定义：
 *    A：三枚硬币面垂直于地面、横向叠放（直径25mm）
 *    B：自选的笔（须与任务五画图用笔一致）
 *    C：自定义物体（主体形状不得为圆柱体和球体）
 *
 *  实现：
 *    1. 串口 A/B/C → 调取对应"抓取点 + 放置点"点位
 *    2. 动作序列（pickAndPlace）：
 *         回待机 → 移到抓取点上方(抬升) → 下降到位 → 闭合夹爪(夹取)
 *         → 抬升 → 平移到放置点上方 → 下降到放置点 → 张开(放下) → 回待机
 *    3. 点位集中定义为可调参数：联调时只需用 Task1.1 的 b/r/f/c 指令
 *       把机械臂移到目标位置，读 ? 的角度填入即可，无需改逻辑。
 *
 *  串口协议（9600 波特率，指令以回车/换行结束）：
 *        A  夹取硬币A并放置到位置1
 *        B  夹取笔B并放置到位置2
 *        C  夹取自定义物体C并放置到位置3
 *        ?  打印当前点位表
 *
 *  硬件接线：舵机 9/8/7/6；夹爪依赖机械方向改进后的夹爪（配合参数标定）
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库（标准库）

/* ---------------- 舵机引脚定义 ---------------- */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6

/* ---------------- 舵机限位 ---------------- */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  30          // 后臂角度下限
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  130         // 前臂角度上限
#define CLAW_MIN  0          // 夹爪张开极限角度
#define CLAW_MAX  120         // 夹爪闭合极限角度

/* ---------------- 运动速度 ---------------- */
int servoDelay = 45;          // 每转 1° 的延时毫秒数（动作执行速度）

/* ---------------- 舵机对象与当前角度 ---------------- */
Servo base, rArm, fArm, claw; // 4 个舵机对象
int curBase = 80;             // 底座当前角度
int curRArm = 30;             // 后臂当前角度
int curFArm = 80;             // 前臂当前角度
int curClaw = 60;             // 夹爪当前角度

/* ==================== 点位定义（按实物标定！） ====================
 * 每个点位 = 4 个关节角度（b=底座 / r=后臂 / f=前臂 / c=夹爪）
 *
 * 标定方法（重要）：
 *   1. 先烧录 Task1.1，用 b/r/f/c 指令把机械臂移到目标位置
 *   2. 发送 ? 读取当前 4 个角度
 *   3. 把数值填到下面的 Pose 常量中
 *
 * 目前为**示例占位值**，未按实物标定前不要用于实际夹取！ */
struct Pose { int b, r, f, c; };   // 点位结构体：4 个关节角度

const Pose HOME  = { 80, 10, 80, 60 };   // 待机位（安全高度、夹爪半开）

const Pose MIDDLE  = { 80, 70, 20, 130 };

/* 抓取点：夹爪到达物体正上方、准备下爪的位置 */
const Pose GRAB_A = { 36,  114, 10, 45 };  // A：硬币（示例值，须标定）
const Pose GRAB_B = { 128, 102, 30, 45 };  // B：笔   （示例值，须标定）
const Pose GRAB_C = { 115,  90, 10, 45 };  // C：自定义物体（示例值，须标定）

/* 放置点：三个明显不同的位置，x/y 方向须与抓取点有明显位移 */


const Pose PLACE_A = { 111, 130, 60, 45 };  // 放置 A 的位置1
const Pose PLACE_B = { 49,  131, 64, 45 };  // 放置 B 的位置2
const Pose PLACE_C = { 80, 100, 20, 45 };  // 放置 C 的位置3




/* 夹取/放开的夹爪角度（方向不对就交换两值） */
const int GRIP_CLOSE = CLAW_MAX;          // 闭合=夹住物体
const int GRIP_OPEN  = CLAW_MIN;          // 张开=放下物体

/* ---------------- 串口行缓冲 ---------------- */
char lineBuf[32];             // 一行指令的字符缓冲
int  lineLen = 0;             // 缓冲中已接收的字符数

/* ==================== 角度限位钳制 ==================== */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 多舵机同步移动 ====================
 * 功能：4 个关节同步逐度运动到目标角度（全部经过限位保护） */
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

/* ==================== 移动到位点 ====================
 * 功能：把机械臂整体移动到某点位（4 关节同时到位） */
void moveTo(const Pose &p, int stepMs){
  syncMove(p.b, p.r, p.f, p.c, stepMs);
}

/* ==================== 夹取并放置（核心动作序列） ====================
 * 功能：完整执行"夹取 → 抬升 → 平移 → 放下"流程
 * 参数：grab 抓取点；place 放置点；lift 抬升量（前臂角度减少值=上抬）
 * 说明：
 *   "高度位"= 抓取点的前臂角度上抬 lift°（避免夹取/移动时碰撞桌面）
 *   步骤间 delay(300) 是让舵机完全到位、物体稳定后再进行下一步 */
void pickAndPlace(const Pose &grab, const Pose &place, int lift = 15){
  Serial.println(">>> 开始夹取-放置动作");

  /* 步骤1：回待机（安全高度，避免启动时撞到物体） */
  moveTo(HOME, servoDelay);

  /* 步骤2：移到抓取点上方（高度位：前臂上抬，防止刮碰物体） */
  int hiR = grab.r;                                 // 高度位后臂角 = 抓取点后臂角
  int hiF = grab.f - lift;                          // 高度位前臂角 = 抓取点前臂上抬 lift°
  syncMove(grab.b, hiR, hiF, GRIP_OPEN, servoDelay);

  /* 步骤3：下降到抓取点（夹爪张开状态） */
  syncMove(grab.b, grab.r, grab.f, GRIP_OPEN, servoDelay);
  delay(1000);                                       // 稳定一下

  /* 步骤4：闭合夹爪（夹取物体） */
  syncMove(grab.b, grab.r, grab.f, GRIP_CLOSE, servoDelay);
  delay(1000);                                       // 确认夹稳

  /* 步骤5：抬升回高度位（带着物体离开桌面） */
  syncMove(grab.b, hiR, hiF, GRIP_CLOSE, servoDelay);

 moveTo(MIDDLE, servoDelay);


  /* 步骤6：平移到放置点上方（底座转动 + 保持高度） */
  syncMove(place.b, hiR, hiF, GRIP_CLOSE, servoDelay);

  /* 步骤7：下降到放置点（带着物体） */
  syncMove(place.b, place.r, place.f, GRIP_CLOSE, servoDelay);
  delay(1000);

  /* 步骤8：张开夹爪（放下物体） */
  syncMove(place.b, place.r, place.f, GRIP_OPEN, servoDelay);
  delay(1000);

  /* 步骤9：回待机，等待下一条指令 */
  moveTo(HOME, servoDelay);
  Serial.println(">>> 动作完成");
}

/* ==================== 指令分发执行 ====================
 * 功能：根据 A/B/C 调取对应抓取点与放置点执行夹取放置 */
void doPickPlace(char obj){
  switch (obj){
    case 'A': pickAndPlace(GRAB_A, PLACE_A); break; // 硬币 A
    case 'B': pickAndPlace(GRAB_B, PLACE_B); break; // 笔 B
    case 'C': pickAndPlace(GRAB_C, PLACE_C); break; // 自定义物体 C
  }
}

/* ==================== 点位表打印 ====================
 * 功能：打印当前点位配置，方便联调时对照修改 */
void reportStatus(){
  Serial.println("----- 点位表（须按实物标定）-----");
  Serial.println(" 待机: b90 r90 f90 c60");
  Serial.print(" 抓取A: b"); Serial.print(GRAB_A.b); Serial.print(" r"); Serial.print(GRAB_A.r);
  Serial.print(" f"); Serial.print(GRAB_A.f); Serial.print(" c"); Serial.println(GRAB_A.c);
  Serial.print(" 放置A: b"); Serial.print(PLACE_A.b); Serial.print(" r"); Serial.print(PLACE_A.r);
  Serial.print(" f"); Serial.print(PLACE_A.f); Serial.print(" c"); Serial.println(PLACE_A.c);
  Serial.println("------------------------------");
}

/* ==================== 串口指令解析 ====================
 * 功能：处理一行串口指令（A/B/C/?，大小写均可） */
void handleCommand(const char *line){
  int len = strlen(line);
  if (len == 0) return;
  char c = toupper(line[0]);                        // 统一大写
  if (len == 1){
    if (c == 'A' || c == 'B' || c == 'C'){          // 夹取放置指令
      Serial.print(">>> 收到指令 "); Serial.println(c);
      doPickPlace(c);                               // 执行对应物体的夹取放置
      return;
    }
    if (c == '?'){                                  // 点位表
      reportStatus();
      return;
    }
  }
  Serial.println("! 未知指令（A/B/C 夹取放置；? 点位表）");
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
  base.attach(PIN_BASE);  delay(200);
  rArm.attach(PIN_RARM);  delay(200);
  fArm.attach(PIN_FARM);  delay(200);
  claw.attach(PIN_CLAW);  delay(200);

  Serial.begin(9600);
  Serial.println("=== Task2 自动夹取与放置：A/B/C ===");
  Serial.println("发送 A/B/C 自动夹取并放置；? 查看点位表");
  Serial.println("注意：点位为示例值，须用 Task1.1 标定后填入！");
}

/* ==================== 主循环 ====================
 * 每轮只做一件事：等待并处理串口指令 */
void loop(){
  readSerialLine();
}
