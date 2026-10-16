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
 *  【运动学】已对齐 JoyFK_alpha2 定稿模型：
 *    正解 forwardKin 与逆解 solveIK 互逆，公式见对应函数注释。
 *    几何参数 L1（底盘高）/L2（后臂）/L3（前臂）按实物标定；
 *    纸面高度 PEN_Z（笔尖 mz）按实际摆放标定（底座与纸同放桌面时 = 0）。
 *
 *  按键协议（第二块板 KeypadBoard_Task5 → 本板，9600 波特率）：
 *    '1' 示教/记录点   '2' 暂停   '3' 继续   '4' 取消
 *    '5' 折线绘制     '6' 曲线绘制   '7' 直线   '8' 基础图形
 *   （另保留原串口调试指令 g/t/s/c/p/x/?，未接按键板时可手动调试）
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
#define RARM_MIN  45          // 后臂角度下限（对齐 JoyFK_alpha2）
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  110         // 前臂角度上限（对齐 JoyFK_alpha2）
#define CLAW_MIN  5           // 夹爪张开极限
#define CLAW_MAX  130         // 夹爪闭合极限（对齐 JoyFK_alpha2）
#define CLAW_HOLD 120         // 夹笔保持角：绘制/示教时夹爪先闭合夹稳铅笔（堵转响则降到 110~100）

/* ---------------- 机械臂几何参数（对齐 JoyFK_alpha2 实物标定） ----------------
 * 运动学模型（JoyFK_alpha2 定稿）：
 *   L1 = 底盘高度：底座顶面 → 肩轴（mm）
 *   L2 = 后臂长度：肩 → 肘（mm）
 *   L3 = 前臂长度：肘 → 腕（mm）
 *   正解：r = L2·cosα − L3·cos(α+β)；mz = L1 + L2·sinα − L3·sin(α+β)
 *   角度映射：θ1 = curBase−90（从 Y 轴量起）、α = 180−curRArm、β = curFArm
 * 铅笔固定于夹爪，抬笔/落笔通过前臂角度上抬 PEN_LIFT 度实现。 */
#define L1  38.0f    // 底盘高度 mm（JoyFK_alpha2 实测）
#define L2  80.0f    // 后臂长度 mm（JoyFK_alpha2 实测）
#define L3  80.0f    // 前臂长度 mm（JoyFK_alpha2 实测）
#define PEN_Z  20.0f // 腕点高度：笔尖触纸时夹爪中心的 mz（mm）= 纸面高度 + 铅笔从夹爪伸出的长度
                     // 标定：纸与底座同放桌面时纸面 mz=0，腕点高 = 直尺量"夹爪中心→笔尖"距离（示例 20，须实测）
#define PEN_LIFT  8  // 抬笔量：腕点高度增量（mm），抬笔时腕点升到 PEN_Z+PEN_LIFT、笔尖离纸
                     // 说明：以腕点抬升 mm 实现（旧版用前臂减角，β 小时被 FARM_MIN 钳制会抬笔失效）

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
  syncMove(90, 90, 90, CLAW_HOLD, servoDelay);      // 回待机时夹爪保持闭合夹笔
}

/* ==================== 运动学正解（对齐 JoyFK_alpha2） ====================
 * 输入：关节几何角 θ1（底座，从 Y 轴量起）、α（后臂仰角）、β（前臂内角），单位度
 * 输出：腕点坐标 mx、my、mz（mm，原点=底座转轴与地面交点）
 * 公式（JoyFK_alpha2 定稿，β 内角·前臂下垂模型；θ1 从 Y 轴量起）：
 *   r  = L2·cosα − L3·cos(α+β)
 *   mx = r·sinθ1，my = r·cosθ1（θ1=0 时腕点在 Y 方向）
 *   mz = L1 + L2·sinα − L3·sin(α+β)   ← L3 项为 −：β<90 前臂下垂、mz 减小 */
void forwardKin(float t1Deg, float aDeg, float bDeg,
                float &mx, float &my, float &mz){
  float t1 = radians(t1Deg);
  float a  = radians(aDeg);
  float b  = radians(bDeg);
  float r  = L2 * cos(a) - L3 * cos(a + b);
  mx = r * sin(t1);                            // X 方向（θ1=0 时为 0）
  my = r * cos(t1);                            // Y 方向（θ1=0 时 = r，小臂沿 Y 延伸）
  mz = L1 + L2 * sin(a) - L3 * sin(a + b);     // 高度（− 号：β 内角，前臂下垂）
}

/* 舵机角 → 几何角（与 JoyFK_alpha2 一致） */
void servoToGeo(float &t1, float &alpha, float &beta){
  t1    = (float)(curBase - 90);
  alpha = (float)(180 - curRArm);
  beta  = (float)curFArm;
}

/* 串口打印当前笔尖正解坐标（调试/示教验证用） */
void printPenPos(){
  float t1, a, b, mx, my, mz;
  servoToGeo(t1, a, b);
  forwardKin(t1, a, b, mx, my, mz);
  Serial.print("  笔尖(x,y,z): ");
  Serial.print(mx, 1); Serial.print(", ");
  Serial.print(my, 1); Serial.print(", ");
  Serial.println(mz, 1);
}

/* ==================== 运动学逆解（与 JoyFK_alpha2 正解互逆） ====================
 * 输入：纸面坐标 (x, y)（x 向前、y 向右，mm）与笔尖高度 z（mz，mm）
 * 输出：底座角 bAng、后臂角 rAng、前臂角 fAng（舵机角度）
 * 原理（正解公式求逆）：
 *   1. 水平投影 R = sqrt(x² + y²)，相对肩轴高度 Z = z − L1
 *   2. 腕点到肩轴距离 d = sqrt(R² + Z²)，超出工作空间返回失败
 *   3. 余弦定理求前臂内角 β：d² = L2² + L3² − 2·L2·L3·cosβ
 *   4. 后臂仰角 α = 腕点极角 φ + 肘部偏移角（atan2(L3·sinβ, L2 − L3·cosβ)）
 *   5. 底座转角 θ1 = atan2(y, x)（与正解 mx=r·sinθ1, my=r·cosθ1 互逆）
 *   6. 舵机角映射（与 servoToGeo 互逆）：base = θ1+90、rArm = 180−α、fArm = β */
bool solveIK(float x, float y, float z, int &bAng, int &rAng, int &fAng){
  float R = sqrt(x * x + y * y);                 // 腕点水平投影距离
  float Z = z - L1;                              // 腕点相对肩轴的竖直高度
  float d = sqrt(R * R + Z * Z);                 // 腕点到肩轴距离
  if (d > L2 + L3 - 1.0f || d < fabs(L2 - L3) + 1.0f){
    return false;                                // 目标点不可达（超出工作空间）
  }
  /* 余弦定理：前臂内角 β（正解 d² = L2² + L3² − 2·L2·L3·cosβ 的逆） */
  float cosB = (L2 * L2 + L3 * L3 - d * d) / (2.0f * L2 * L3);
  if (cosB > 1.0f) cosB = 1.0f;                  // 数值保护
  if (cosB < -1.0f) cosB = -1.0f;
  float beta  = acos(cosB);
  /* 后臂仰角 α = 腕点极角 + 肘部偏移角 */
  float phi   = atan2(Z, R);                     // 腕点相对肩的极角
  float alpha = phi + atan2(L3 * sin(beta), L2 - L3 * cos(beta));
  /* 奇异检测：腕点水平投影若越过肩轴正下方/后方（肘朝后姿态，r≈0 或为负），
   * 纸面坐标失去唯一方向，返回不可达，避免画出错误动作 */
  float rChk = L2 * cos(alpha) - L3 * cos(alpha + beta);
  if (rChk < 5.0f) return false;
  /* 底座转角 θ1（纸面 y 向右 = mx、x 向前 = my） */
  float t1 = atan2(y, x);
  /* 几何角(rad) → 舵机角(度)，与 servoToGeo 互逆 */
  bAng = (int)round(degrees(t1) + 90.0f);
  rAng = (int)round(180.0f - degrees(alpha));
  fAng = (int)round(degrees(beta));

  bAng = clampVal(bAng, BASE_MIN, BASE_MAX);    // 底座限位
  rAng = clampVal(rAng, RARM_MIN, RARM_MAX);    // 后臂限位
  fAng = clampVal(fAng, FARM_MIN, FARM_MAX);    // 前臂限位

  /* 回代验证：限位钳制可能让目标实际不可达（如双解中底座被钳到极限），
   * 用正解核对腕点是否真的到达 (x, y)；误差 > 5mm 判定不可达 */
  float vT1  = radians((float)bAng - 90.0f);
  float vAlp = radians(180.0f - (float)rAng);
  float vBet = radians((float)fAng);
  float vr   = L2 * cos(vAlp) - L3 * cos(vAlp + vBet);
  float vmx  = vr * sin(vT1);                    // 对应纸面 y（向右）
  float vmy  = vr * cos(vT1);                    // 对应纸面 x（向前）
  if (sqrt((vmy - x) * (vmy - x) + (vmx - y) * (vmx - y)) > 5.0f){
    return false;                                // 钳制后到不了目标
  }
  return true;
}

/* ==================== 笔尖移动到纸面坐标 ====================
 * 功能：把笔尖移动到纸面坐标 (x, y)（纸面高度 PEN_Z），并按 down 决定抬笔/落笔
 * 说明：抬笔 = 前臂上抬 PEN_LIFT°（笔尖离开纸面），落笔 = 前臂到位 */
bool movePenTo(float x, float y, bool down){
  int b, r, f;
  float zT = down ? PEN_Z : PEN_Z + PEN_LIFT;       // 落笔=腕点贴纸面；抬笔=腕点抬升 PEN_LIFT mm
  if (!solveIK(x, y, zT, b, r, f)){                 // 逆解失败（不可达）
    Serial.print("! 目标不可达 ("); Serial.print(x); Serial.print(",");
    Serial.print(y); Serial.println(")");
    return false;
  }
  syncMove(b, r, f, CLAW_HOLD, servoDelay);         // 夹爪闭合夹稳铅笔
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
  Point2D a = { 40, 0 }, b = { 70, 0 }, c = { 55, 32 };  // 前臂竖直下垂区三角形（画线中形态稳定、不抬升）
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
  centerAll();                                      // 先回待机位（考核要求：先回待机再移向第1点）
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
  centerAll();                                      // 先回待机位（考核要求：先回待机再移向第1点）
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
  teachPts[0] = { 50.0f, 0.0f };                    // 预置可达起点（(0,0)在肩轴正上方不可达）
  state = S_TEACH;
  movePenTo(50, 0, false);                          // 笔尖先移到可达示教起点
  Serial.println(">>> 示教模式：笔尖已到起点(50,0)；摇杆移动笔尖，按键1单击记录（最多5点）；按键3双击=折线 / 三击=曲线");
  Serial.println("    注意：只能往机械臂前方/两侧推（x 保持正值）；方向反了就调 DIR_JOY1_X/Y");
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
      float nx = teachPts[teachCount - 1].x + dX * step; // 目标坐标
      float ny = teachPts[teachCount - 1].y + dY * step;
      if (movePenTo(nx, ny, false)){                // 可达才移动
        teachPts[teachCount - 1].x = nx;            // 更新临时点 x
        teachPts[teachCount - 1].y = ny;            // 更新临时点 y
      } else {
        Serial.println("! 该方向越界不可达：反向推，或调 DIR_JOY1_X/Y 方向系数");
      }
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
 * 功能：处理一行指令（按键板指令 '1'~'8'；另保留 g/t/s/c/p/x/? 调试）
 * 按键映射（与 KeypadBoard_Task5.ino 一致）：
 *   '1' 示教/记录   '2' 暂停   '3' 继续   '4' 取消
 *   '5' 折线       '6' 曲线   '7' 直线   '8' 基础图形 */
void handleCommand(const char *line){
  int len = strlen(line);
  if (len == 0) return;
  char c = tolower(line[0]);                        // 统一小写
  switch (c){
    case '1':                                       /* 按键1：示教/记录 */
    case 't':                                       /* 调试：示教/记录 */
      if (state != S_TEACH) enterTeach();           // 未在示教 → 进入
      else {                                        // 已在示教 → 记录当前点
        if (teachCount < 5){
          if (teachCount > 0){
            teachPts[teachCount] = teachPts[teachCount - 1]; // 新槽继承当前点（摇杆连续移动不跳变）
          }
          teachCount++;                             // 确认记录一个点
          Serial.print(">>> 已记录示教点 "); Serial.println(teachCount);
        } else {
          Serial.println("! 已达 5 点上限");
        }
      }
      break;
    case '2':                                       /* 按键：暂停 */
      if (state == S_DRAWING){ state = S_PAUSED; Serial.println(">>> 已暂停"); }
      break;
    case '3':                                       /* 按键：继续 */
      if (state == S_PAUSED){ state = S_DRAWING; Serial.println(">>> 已继续"); }
      break;
    case '4':                                       /* 按键：取消 */
    case 'x':                                       /* 调试：取消 */
      state = S_IDLE;
      Serial.println(">>> 已取消，回待机");
      centerAll();                                  // 回中待机位
      break;
    case '5':                                       /* 按键：折线绘制 */
    case 's':                                       /* 调试：折线 */
      if (state == S_TEACH){ state = S_DRAWING; drawPolyline(); state = S_IDLE; }
      break;
    case '6':                                       /* 按键：曲线绘制 */
    case 'c':                                       /* 调试：曲线 */
      if (state == S_TEACH){ state = S_DRAWING; drawCurve(); state = S_IDLE; }
      break;
    case '7':                                       /* 按键：直线演示 */
    case 'g':                                       /* 调试：直线 */
      state = S_DRAWING;
      Serial.println(">>> 直线绘制演示");
      movePenTo(40, 0, false);                      // 抬笔到起点（前臂竖直下垂区，避免画线中翻转抬升）
      drawLineTo((Point2D){ 40, 0 }, (Point2D){ 70, 0 }); // 画直线
      state = S_IDLE;                               // 完成后回待机
      break;
    case '8':                                       /* 按键：基础图形 */
      state = S_DRAWING;
      Serial.println(">>> 基础图形绘制（三角形）");
      demoDrawTriangle();                           // 画三角形
      state = S_IDLE;
      break;
    case 'p':                                       /* 调试：暂停/继续（切换） */
      if (state == S_DRAWING){ state = S_PAUSED; Serial.println(">>> 已暂停"); }
      else if (state == S_PAUSED){ state = S_DRAWING; Serial.println(">>> 已继续"); }
      break;
    case '?':                                       /* 状态查询 */
      Serial.print("状态:"); Serial.print(state);
      Serial.print(" 示教点数:"); Serial.print(teachCount);
      printPenPos();                                // 正解显示当前笔尖坐标
      break;
    default:                                        /* 未知指令 */
      Serial.println("! 指令：1示教/记录 2暂停 3继续 4取消 5折线 6曲线 7直线 8图形 | 调试 g/t/s/c/p/x/?");
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

  base.write(90); rArm.write(90); fArm.write(90); claw.write(CLAW_HOLD);  // 上电先闭合夹爪夹笔
  delay(300);

  Serial.begin(9600);
  Serial.println("=== Task5 轨迹绘制 ===");
  Serial.println("运动学：对齐 JoyFK_alpha2；PEN_Z=腕点高度(笔尖触纸时)按实物标定；绘制中夹爪闭合夹笔");
  Serial.println("按键：1示教/记录 2暂停 3继续 4取消 5折线 6曲线 7直线 8图形（调试 g/t/s/c/p/x/?）");
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
