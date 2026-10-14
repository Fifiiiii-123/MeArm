/*
 * ============================================================================
 *  文件：Task5.ino
 *  工程：Task5 —— 机械臂轨迹绘制与画图（考核方向内 35%，难度分累计）
 * ----------------------------------------------------------------------------
 *  子任务（难度分）：
 *    1. 直线绘制（6）
 *    2. 基础图形：N / 三角形 / Z / V 任选一种（10）
 *    3. 绘图过程控制：暂停/继续/取消，按键触发（24）
 *    4. 五点示教与折线轨迹规划（30）
 *    5. 五点示教与平滑曲线轨迹规划（30）
 *
 *  总体流程：
 *    摇杆示教记录目标点 → 按键/指令启动自动绘制 →
 *    运动学逆解把纸面坐标转成关节角 → 插补逐点执行
 *
 *  【重要】本工程为"可编译骨架"：直线/折线/曲线插补框架与状态机完整，
 *  但 `solveIK()` 依赖任务四推导的运动学公式。当前为两连杆几何逆解
 *  （基于 L1/L2 默认长度），联调时须：
 *    1. 实测并填入 L1（后臂）、L2（前臂）长度（毫米）
 *    2. 若关节角与几何角映射不同，修正 IK 输出到舵机角的映射
 *
 *  串口协议（9600 波特率）：
 *    g  直线演示      t 示教模式/记录点    s 折线绘制    c 曲线绘制
 *    p  暂停/继续     x 取消并回中         ? 状态
 *
 *  硬件接线：
 *    舵机：base=D9  rArm=D8  fArm=D7  claw=D6（夹爪夹持铅笔）
 *    摇杆：A3/A2/A1/A0（示教模式移动笔尖）
 * ============================================================================
 */
#include <Servo.h>            // Arduino 舵机驱动库（标准库）
#include <math.h>             // 数学库：acos/atan2/sqrt/degrees 等

/* ---------------- 引脚定义 ---------------- */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6

/* ---------------- 摇杆模拟引脚 ---------------- */
#define JOY1_X    A3          // 右摇杆上下 → 示教时移动 y
#define JOY1_Y    A2          // 右摇杆左右 → 示教时移动 x
#define JOY2_X    A1          // 左摇杆上下 → 预留
#define JOY2_Y    A0          // 左摇杆左右 → 预留

/* ---------------- 舵机限位 ---------------- */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  30          // 后臂角度下限
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  130         // 前臂角度上限
#define CLAW_MIN  5           // 夹爪张开极限
#define CLAW_MAX  100         // 夹爪闭合极限

/* ---------------- 机械臂几何参数（须按实物标定！） ----------------
 * L1/L2 是后臂、前臂的长度（毫米），决定逆解结果。
 * 铅笔固定于夹爪，抬笔/落笔通过前臂角度上抬 PEN_LIFT 度实现。 */
#define L1  70.0f    // 后臂长度 mm（示例值，须实测！）
#define L2  80.0f    // 前臂长度 mm（示例值，须实测！）
#define PEN_LIFT  18 // 抬笔/落笔对应的前臂角度增量（示例值，须标定）

/* ---------------- 摇杆参数 ---------------- */
#define JOY_CENTER   512      // 摇杆模拟值中点
#define JOY_DEADBAND 20       // 死区
#define JOY_MAX_STEP 1        // 单轮最大增量

/* ---------------- 摇杆方向配置（示教模式纸面方向） ----------------
 * 示教模式用摇杆移动纸面坐标（JOY1_X=右摇杆上下→纸面x，JOY1_Y=右摇杆
 * 左右→纸面y）。方向系数当前为占位 +1，联调示教时若摇杆推的方向与
 * 笔尖移动相反，只需把对应系数改为 -1，不动运行逻辑。
 *   右摇杆上下 → 纸面 x(+1 待标定)    右摇杆左右 → 纸面 y(+1 待标定) */
#define DIR_JOY1_X  +1         // 纸面 x 方向系数（待实物标定）
#define DIR_JOY1_Y  +1         // 纸面 y 方向系数（待实物标定）
#define DIR_JOY2_X  +1         // 预留
#define DIR_JOY2_Y  +1         // 预留

/* ---------------- 运动速度 ---------------- */
int servoDelay = 8;           // 绘制时每度延时（越小越快，注意机械响应）

/* ---------------- 舵机对象与当前角度 ---------------- */
Servo base, rArm, fArm, claw; // 4 个舵机对象
int curBase = 90;             // 底座当前角度
int curRArm = 90;             // 后臂当前角度
int curFArm = 90;             // 前臂当前角度
int curClaw = 90;             // 夹爪当前角度

/* ==================== 状态机 ====================
 * 绘制任务的运行状态：
 *   S_IDLE    待机（等待指令）
 *   S_TEACH   示教模式（摇杆移动笔尖 + t 记录点）
 *   S_DRAWING 自动绘制中
 *   S_PAUSED  暂停（保持当前位置与任务状态）
 */
enum State { S_IDLE, S_TEACH, S_DRAWING, S_PAUSED };
State state = S_IDLE;         // 当前状态

/* ==================== 示教点数据 ====================
 * 纸面坐标（mm，原点=肩关节投影），最多 5 个目标点 */
struct Point2D { float x, y; };          // 二维纸面坐标
Point2D teachPts[5];                     // 示教点数组
int  teachCount = 0;                     // 已记录的示教点数
bool penDown = false;                    // 笔尖当前是否落笔（接触纸面）

/* ---------------- 串口行缓冲 ---------------- */
char lineBuf[32];
int  lineLen = 0;

/* ==================== 角度限位钳制 ==================== */
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

/* ==================== 回中 ====================
 * 功能：所有关节回到 90° 初始姿态 */
void centerAll(){
  syncMove(90, 90, 90, 90, servoDelay);
}

/* ==================== 运动学逆解（核心，依赖任务四） ====================
 * 输入：末端纸面坐标 (x, y)，肩关节为原点，x 向前、y 向右（右手系，按实物定义）
 * 输出：底座角 bAng、后臂角 rAng、前臂角 fAng（均换算成舵机角度）
 * 实现：平面两连杆几何逆解（任务四推导的代码版）
 *
 * 原理简述：
 *   1. 末端到肩关节的距离 dist = sqrt(x² + y²)，若超出工作空间返回失败
 *   2. 余弦定理求肘关节角 theta2（前臂相对后臂的夹角）
 *   3. 由末端极角减去前臂偏移角得到肩关节角 theta1（后臂角）
 *   4. 几何角(rad) → 舵机角(度) 的映射为占位公式，须按实物修正！
 */
bool solveIK(float x, float y, int &bAng, int &rAng, int &fAng){
  float dist = sqrt(x * x + y * y);                 // 末端到肩关节距离
  if (dist > L1 + L2 - 1.0f || dist < fabs(L1 - L2) + 1.0f){
    return false;                                   // 目标点不可达（超出工作空间）
  }
  /* 余弦定理：前臂相对后臂的夹角 cosVal → theta2 */
  float cosVal = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (cosVal > 1.0f) cosVal = 1.0f;                 // 数值保护
  if (cosVal < -1.0f) cosVal = -1.0f;
  float theta2 = acos(cosVal);                      // 肘关节角（前臂相对后臂延长方向）

  /* 肩关节角：末端极角 - 前臂引起的角度偏移 */
  float beta  = atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
  float theta1 = atan2(y, x) - beta;                // 后臂相对基准线(竖直)的角度

  /* 几何角(rad) → 舵机角(度)（占位映射，按实物方向修正后使用！）：
   *   后臂舵机：竖直为 90°，向前摆角 → rAng ≈ 90 - degrees(theta1)
   *   前臂舵机：后臂延长方向为 90° → fAng ≈ 90 - degrees(theta2)
   *   底座舵机：x-y 平面极角 → bAng ≈ degrees(atan2(y, x)) 映射 0~180 */
  bAng = (int)round(degrees(atan2(y, x)));
  rAng = (int)round(90 - degrees(theta1));
  fAng = (int)round(90 - degrees(theta2));

  bAng = clampVal(bAng, BASE_MIN, BASE_MAX);        // 底座限位
  rAng = clampVal(rAng, RARM_MIN, RARM_MAX);        // 后臂限位
  fAng = clampVal(fAng, FARM_MIN, FARM_MAX);        // 前臂限位
  return true;
}

/* ==================== 笔尖移动到纸面坐标 ====================
 * 功能：把笔尖移动到纸面坐标 (x, y)，并按 down 决定抬笔/落笔
 * 说明：抬笔 = 前臂上抬 PEN_LIFT°（笔尖离开纸面），落笔 = 前臂到位 */
bool movePenTo(float x, float y, bool down){
  int b, r, f;
  if (!solveIK(x, y, b, r, f)){                     // 逆解失败（不可达）
    Serial.print("! 目标不可达 ("); Serial.print(x); Serial.print(",");
    Serial.print(y); Serial.println(")");
    return false;
  }
  int fTarget = down ? f : f - PEN_LIFT;            // 落笔用前臂角，抬笔上抬
  syncMove(b, r, fTarget, 90, servoDelay);          // 夹爪保持 90° 夹笔
  penDown = down;                                   // 记录落笔状态
  return true;
}

/* ==================== 插补绘制参数 ==================== */
#define DRAW_STEPS 40   // 线段插补步数（每段直线分 40 步逼近）

/* ==================== 两点直线插补绘制 ====================
 * 功能：在 p0→p1 之间画一条连续直线（先抬笔过去、落笔后逐点画）
 * 说明：每步之间检查状态——暂停则原地等待、取消则立即退出 */
void drawLineTo(Point2D p0, Point2D p1){
  movePenTo(p0.x, p0.y, false);                     // 抬笔移到起点
  movePenTo(p0.x, p0.y, true);                      // 落笔
  for (int i = 1; i <= DRAW_STEPS; i++){
    if (state == S_PAUSED){                         // 暂停：原地等待（期间仍处理串口）
      while (state == S_PAUSED) { readSerialLine(); delay(20); }
    }
    if (state != S_DRAWING) return;                 // 取消：退出绘制
    float t = (float)i / DRAW_STEPS;                // 插值比例 0~1
    float x = p0.x + (p1.x - p0.x) * t;             // x 线性插值
    float y = p0.y + (p1.y - p0.y) * t;             // y 线性插值
    if (!movePenTo(x, y, true)) break;              // 中途不可达则停止
  }
}

/* ==================== 基础图形演示：三角形 ====================
 * 功能：画一个三角形（直线插补的调用示例，满足子任务2）
 * 说明：子任务2 可从 N/三角形/Z/V 中任选一种，改三个顶点即可换图形 */
void demoDrawTriangle(){
  Point2D a = { -40, 0 }, b = { 40, 0 }, c = { 0, 50 };
  drawLineTo(a, b);                                 // 边1
  drawLineTo(b, c);                                 // 边2
  drawLineTo(c, a);                                 // 边3
  movePenTo(a.x, a.y, false);                       // 画完抬笔
}

/* ==================== 五点折线绘制 ====================
 * 功能：按示教顺序依次连接 5 个点（满足子任务4）
 * 说明：每段用 drawLineTo 直线插补；点数不足 2 时提示 */
void drawPolyline(){
  if (teachCount < 2){                              // 点数不足
    Serial.println("! 示教点不足，先进入示教模式记录点");
    return;
  }
  Serial.println(">>> 折线绘制开始");
  for (int i = 0; i + 1 < teachCount; i++){         // 逐段连接
    if (state != S_DRAWING) break;                  // 被取消则中断
    drawLineTo(teachPts[i], teachPts[i + 1]);
  }
  Serial.println(">>> 折线绘制结束");
}

/* ==================== 五点平滑曲线绘制 ====================
 * 功能：按示教顺序绘制经过 5 点的连续曲线（满足子任务5）
 * 实现：Catmull-Rom 样条骨架，每段用 SEGS 段线段逼近；
 *       联调时若轨迹不光滑可增大 SEGS 细分 */
void drawCurve(){
  if (teachCount < 3){                              // 曲线至少 3 点
    Serial.println("! 平滑曲线至少需要 3 个点");
    return;
  }
  Serial.println(">>> 曲线绘制开始");
  const int SEGS = 24;                              // 每段细分步数
  for (int i = 0; i + 3 < teachCount + 2; i++){     // 遍历曲线段
    if (state != S_DRAWING) break;                  // 被取消则中断
    /* 取相邻 4 个控制点（首尾自动补全） */
    Point2D p0 = (i - 1 >= 0) ? teachPts[i - 1] : teachPts[0];
    Point2D p1 = teachPts[i];
    Point2D p2 = (i + 1 < teachCount) ? teachPts[i + 1] : teachPts[teachCount - 1];
    Point2D p3 = (i + 2 < teachCount) ? teachPts[i + 2] : teachPts[teachCount - 1];
    float prevX = p1.x, prevY = p1.y;               // 段起点
    for (int s = 1; s <= SEGS; s++){                // 细分逼近
      if (state == S_PAUSED){                       // 暂停等待
        while (state == S_PAUSED) { readSerialLine(); delay(20); }
      }
      if (state != S_DRAWING) return;               // 取消退出
      float t = (float)s / SEGS;                    // 本段插值比例
      float t2 = t * t, t3 = t2 * t;
      /* Catmull-Rom 基函数求曲线点 (x, y) */
      float x = 0.5f * ((2 * p1.x) + (-p0.x + p2.x) * t +
                        (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                        (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
      float y = 0.5f * ((2 * p1.y) + (-p0.y + p2.y) * t +
                        (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                        (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
      Point2D np = { x, y };                        // 当前目标点
      if (s == 1) movePenTo(prevX, prevY, false);   // 段首抬笔落位
      movePenTo(np.x, np.y, true);                  // 落笔画点
    }
  }
  Serial.println(">>> 曲线绘制结束");
}

/* ==================== 示教模式 ====================
 * 功能：进入示教模式，提示用户用摇杆移动笔尖、发 t 记录点 */
void enterTeach(){
  teachCount = 0;                                   // 清空示教点
  state = S_TEACH;
  Serial.println(">>> 示教模式：用摇杆把笔尖移到目标点，发 t 记录（最多5点）；发 s/c 开始绘制");
}

/* ==================== 示教摇杆操控 ====================
 * 功能：示教模式下用摇杆移动笔尖（每次 ±2mm 步进）
 * 说明：当前为骨架实现——直接修改最后记录点的坐标并移动；
 *       联调建议：用任务四正解反推当前笔尖坐标，实现更准确的示教 */
void joystickTeach(){
  int dX = DIR_JOY1_X * axisDelta(JOY1_X);          // 纸面 x 移动增量（方向系数见配置区）
  int dY = DIR_JOY1_Y * axisDelta(JOY1_Y);          // 纸面 y 移动增量（方向系数见配置区）
  float step = 2.0f;                                // 每次移动 2mm
  if (dX || dY){
    if (teachCount > 0){                            // 有记录点才移动
      teachPts[teachCount - 1].x += dX * step;      // 更新临时点 x
      teachPts[teachCount - 1].y += dY * step;      // 更新临时点 y
      movePenTo(teachPts[teachCount - 1].x, teachPts[teachCount - 1].y, false);
    }
    delay(20);                                      // 移动节拍
  }
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

/* ==================== 串口指令解析 ====================
 * 功能：处理一行串口指令（g/t/s/c/p/x/?）
 *  g 直线演示   t 示教/记录   s 折线   c 曲线   p 暂停/继续   x 取消 */
void handleCommand(const char *line){
  int len = strlen(line);
  if (len == 0) return;
  char c = tolower(line[0]);                        // 统一小写
  switch (c){
    case 'g':                                       /* 直线演示 */
      state = S_DRAWING;
      Serial.println(">>> 直线绘制演示");
      movePenTo(-30, 0, false);                     // 抬笔到起点
      drawLineTo((Point2D){ -30, 0 }, (Point2D){ 30, 20 }); // 画直线
      state = S_IDLE;                               // 完成后回待机
      break;
    case 't':                                       /* 示教模式（t=进入/记录） */
      if (state != S_TEACH) enterTeach();           // 未在示教 → 进入
      else {                                        // 已在示教 → 记录当前点
        if (teachCount < 5){
          teachCount++;                             // 确认记录一个点
          Serial.print(">>> 已记录示教点 "); Serial.println(teachCount);
        } else {
          Serial.println("! 已达 5 点上限");
        }
      }
      break;
    case 's':                                       /* 折线绘制 */
      if (state == S_TEACH){ state = S_DRAWING; drawPolyline(); state = S_IDLE; }
      break;
    case 'c':                                       /* 曲线绘制 */
      if (state == S_TEACH){ state = S_DRAWING; drawCurve(); state = S_IDLE; }
      break;
    case 'p':                                       /* 暂停/继续（切换） */
      if (state == S_DRAWING){ state = S_PAUSED; Serial.println(">>> 已暂停"); }
      else if (state == S_PAUSED){ state = S_DRAWING; Serial.println(">>> 已继续"); }
      break;
    case 'x':                                       /* 取消并回待机 */
      state = S_IDLE;
      Serial.println(">>> 已取消，回待机");
      centerAll();                                  // 回中待机位
      break;
    case '?':                                       /* 状态查询 */
      Serial.print("状态:"); Serial.print(state);
      Serial.print(" 示教点数:"); Serial.println(teachCount);
      break;
    default:                                        /* 未知指令 */
      Serial.println("! 指令：g直线 | t示教/记录 | s折线 | c曲线 | p暂停/继续 | x取消");
      break;
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

/* ==================== 初始化 ==================== */
void setup(){
  base.attach(PIN_BASE);  delay(200);
  rArm.attach(PIN_RARM);  delay(200);
  fArm.attach(PIN_FARM);  delay(200);
  claw.attach(PIN_CLAW);  delay(200);

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90);
  delay(300);

  Serial.begin(9600);
  Serial.println("=== Task5 轨迹绘制（骨架）===");
  Serial.println("注意：solveIK 为默认几何模型，须按实物标定 L1/L2 与角度映射！");
  Serial.println("指令：g直线 | t示教/记录 | s折线 | c曲线 | p暂停/继续 | x取消 | ?状态");
}

/* ==================== 主循环 ====================
 * 每轮：
 *   1. 处理串口指令
 *   2. 若处于示教模式：响应摇杆移动笔尖 */
void loop(){
  readSerialLine();                                 // 串口指令

  if (state == S_TEACH){                            // 示教模式
    joystickTeach();                                // 摇杆移动笔尖
  }
}
