/*
 * Task2_alpha2 - JoyFK_alpha2 基础上的任务二：摇杆示教 + A/B/C 自动抓放。
 * 接线、限位、摇杆方向、腕点正解完全沿用 JoyFK_alpha2。
 * 坐标单位 mm，角度单位度。运行时回放舵机角度，不使用逆解。
 *
 * 串口 9600，所有命令以回车或换行结束（大小写均可）：
 *   A / B / C           执行一次对应抓放，完成后回 HOME
 *   X                   中止自动动作，保持当前角度；M 恢复摇杆
 *   M                   待机时启用摇杆（开机默认启用）
 *   ? / TABLE           显示全部点位、坐标、标定状态
 *   POS                 显示当前坐标与 b/r/f/c 四个舵机角度
 *   LIVE                开关每 250ms 的当前坐标输出
 *   SET HOME            记录当前姿态为待机位
 *   SET A PICK          记录 A 抓取点（B/C 同理）
 *   SET A PICKUP        记录 A 抓取点上方点
 *   SET A PLACE         记录 A 放置点
 *   SET A PLACEUP       记录 A 放置点上方点
 *   SET A OPEN          当前夹爪角度作为 A 张开角度
 *   SET A CLOSE         当前夹爪角度作为 A 夹紧角度
 *   H / L               加速 / 减速（同时影响摇杆与自动移动）
 *
 * 标定：用摇杆到达每个目标，发送 SET ... 记录；坐标由正解自动计算。
 * 这些记录仅保存在 RAM，复位/断电丢失。用 TABLE 读取角度，填回下面
 * PLACEHOLDERS 和 HOME_ANGLES，再将对应 INITIAL_TAUGHT 改为 15 后烧录，
 * 即可固化点位。OPEN/CLOSE 同样填回占位表。不要把串口坐标手工反算角度。
 * 默认点全部是占位值；四个位置未完成示教时拒绝执行该物体的 A/B/C。
 * PICKUP/PLACEUP 应实测高于对应点，并检查整段运动的避障空间。
 * 正解输出是腕点而非夹持中心；舵机角度是指令值，无位置传感器反馈。
 */
#include <Servo.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#define PIN_BASE 9
#define PIN_RARM 8
#define PIN_FARM 7
#define PIN_CLAW 6
#define JOY1_X A3
#define JOY1_Y A2
#define JOY2_X A1
#define JOY2_Y A0
#define BASE_MIN 0
#define BASE_MAX 180
#define RARM_MIN 45
#define RARM_MAX 160
#define FARM_MIN 30
#define FARM_MAX 110
#define CLAW_MIN 5
#define CLAW_MAX 130
#define L1 38.0f
#define L2 80.0f
#define L3 80.0f
#define JOY_CENTER 512
#define JOY_DEADBAND 25
#define JOY_MAX_STEP 1
#define DIR_JOY1_X -1
#define DIR_JOY1_Y -1
#define DIR_JOY2_X +1
#define DIR_JOY2_Y +1
#define PRINT_MS 250UL

struct Angles { int b, r, f, c; };
struct Point { Angles angle; float x, y, z; };
enum PointIndex { PICK, PICKUP, PLACE, PLACEUP };
struct ObjectPlan {
  Point point[4];
  int openAngle, closeAngle;
  byte taught;
};

// ==================== 占位参数区：实物标定后替换 ====================
// 每行依次 PICK、PICKUP、PLACE、PLACEUP；每组为 {底座,上臂,前臂,夹爪}。
// 点位中的 c 用于完整记录；抓放时使用独立的 OPEN/CLOSE 角度覆盖 c。
const Angles HOME_ANGLES = {90, 90, 90, 90};
const Angles PLACEHOLDERS[3][4] = {
  {{36,114,45,5}, {36,100,45,5}, {111,130,60,5}, {111,110,60,5}},
  {{128,102,45,5}, {128,90,45,5}, {49,131,64,5}, {49,110,64,5}},
  {{115,90,45,5}, {115,80,45,5}, {80,100,45,5}, {80,85,45,5}}
};
const int OPEN_ANGLES[3] = {5,5,5};
const int CLOSE_ANGLES[3] = {120,120,120};
// 每物体四位标记：PICK=1、PICKUP=2、PLACE=4、PLACEUP=8；全标定为15。
const byte INITIAL_TAUGHT[3] = {0,0,0};
// ==================================================================

Servo base, rArm, fArm, claw;
Angles current = {90,90,90,90};
Point home;
ObjectPlan plans[3];
int servoDelay = 20;
bool manualEnabled = true, liveOutput = true, running = false;
byte activeObject = 0, stage = 0;
Angles startAngle, targetAngle;
int moveSteps = 0, moveProgress = 0;
bool settling = false;
unsigned long lastStep = 0, settleSince = 0, lastPrint = 0, lastManual = 0;
char lineBuf[48];
byte lineLen = 0;
bool lineOverflow = false;

int clampVal(int value, int lo, int hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}
bool validAngles(const Angles &a) {
  return a.b >= BASE_MIN && a.b <= BASE_MAX &&
         a.r >= RARM_MIN && a.r <= RARM_MAX &&
         a.f >= FARM_MIN && a.f <= FARM_MAX &&
         a.c >= CLAW_MIN && a.c <= CLAW_MAX;
}
// 与 JoyFK_alpha2 完全相同：theta=b-90, alpha=180-r, beta=f。
Point makePoint(const Angles &a) {
  Point p;
  p.angle = a;
  float theta = radians((float)(a.b - 90));
  float alpha = radians((float)(180 - a.r));
  float beta = radians((float)a.f);
  float radius = L2 * cos(alpha) - L3 * cos(alpha + beta);
  p.x = radius * sin(theta);
  p.y = radius * cos(theta);
  p.z = L1 + L2 * sin(alpha) - L3 * sin(alpha + beta);
  return p;
}
void writeCurrent() {
  base.write(current.b); rArm.write(current.r);
  fArm.write(current.f); claw.write(current.c);
}
void printPoint(const Point &p) {
  Serial.print(F("b=")); Serial.print(p.angle.b);
  Serial.print(F(" r=")); Serial.print(p.angle.r);
  Serial.print(F(" f=")); Serial.print(p.angle.f);
  Serial.print(F(" c=")); Serial.print(p.angle.c);
  Serial.print(F(" | wrist mm: x=")); Serial.print(p.x, 1);
  Serial.print(F(" y=")); Serial.print(p.y, 1);
  Serial.print(F(" z=")); Serial.println(p.z, 1);
}
void printPointName(byte index) {
  switch (index) {
    case PICK: Serial.print(F("PICK")); break;
    case PICKUP: Serial.print(F("PICKUP")); break;
    case PLACE: Serial.print(F("PLACE")); break;
    case PLACEUP: Serial.print(F("PLACEUP")); break;
  }
}
void reportTable() {
  Serial.println(F("RAM calibration; reset loses SET changes. Copy angles to source to retain."));
  Serial.print(F("HOME ")); printPoint(home);
  for (byte i = 0; i < 3; ++i) {
    Serial.print((char)('A' + i));
    Serial.print(F(" taught=")); Serial.print(plans[i].taught);
    Serial.print(F("/15 open=")); Serial.print(plans[i].openAngle);
    Serial.print(F(" close=")); Serial.println(plans[i].closeAngle);
    for (byte j = 0; j < 4; ++j) {
      printPointName(j); Serial.print(' '); printPoint(plans[i].point[j]);
    }
  }
}
int axisDelta(int pin) {
  int value = analogRead(pin);
  if (value > JOY_CENTER + JOY_DEADBAND) return JOY_MAX_STEP;
  if (value < JOY_CENTER - JOY_DEADBAND) return -JOY_MAX_STEP;
  return 0;
}
void updateJoystick() {
  if (!manualEnabled || running || millis() - lastManual < (unsigned long)servoDelay) return;
  lastManual = millis();
  current.b = clampVal(current.b + DIR_JOY1_Y * axisDelta(JOY1_Y), BASE_MIN, BASE_MAX);
  current.r = clampVal(current.r + DIR_JOY1_X * axisDelta(JOY1_X), RARM_MIN, RARM_MAX);
  current.f = clampVal(current.f + DIR_JOY2_X * axisDelta(JOY2_X), FARM_MIN, FARM_MAX);
  current.c = clampVal(current.c + DIR_JOY2_Y * axisDelta(JOY2_Y), CLAW_MIN, CLAW_MAX);
  writeCurrent();
}
void abortMotion() {
  running = false;
  manualEnabled = false;
  settling = false;
  Serial.println(F("STOP: holding current angles; M enables joystick."));
}
// 相同比例插值三个关节，最大角差的关节每步1度，其余关节同时到达。
// 这是关节空间插值，不保证腕点沿直线运动。
bool beginMove(const Angles &goal) {
  if (!validAngles(goal)) {
    abortMotion(); Serial.println(F("ERROR: target outside limits.")); return false;
  }
  startAngle = current;
  targetAngle = goal;
  moveSteps = 0;
  int differences[4] = {abs(goal.b-current.b), abs(goal.r-current.r),
                        abs(goal.f-current.f), abs(goal.c-current.c)};
  for (byte i=0; i<4; ++i) if (differences[i] > moveSteps) moveSteps = differences[i];
  moveProgress = 0;
  settling = false;
  lastStep = millis();
  return true;
}
void beginStage() {
  ObjectPlan &p = plans[activeObject];
  Angles goal = current;
  // 起步保留夹爪角度，抵达抓取上方后再张开；放下后先向上撤离。
  switch (stage) {
    case 0: goal = home.angle; goal.c = current.c; break;
    case 1: goal = p.point[PICKUP].angle; goal.c = current.c; break;
    case 2: goal.c = p.openAngle; break;
    case 3: goal = p.point[PICK].angle; goal.c = p.openAngle; break;
    case 4: goal.c = p.closeAngle; break;
    case 5: goal = p.point[PICKUP].angle; goal.c = p.closeAngle; break;
    case 6: goal = p.point[PLACEUP].angle; goal.c = p.closeAngle; break;
    case 7: goal = p.point[PLACE].angle; goal.c = p.closeAngle; break;
    case 8: goal.c = p.openAngle; break;
    case 9: goal = p.point[PLACEUP].angle; goal.c = p.openAngle; break;
    case 10: goal = home.angle; goal.c = p.openAngle; break;
    default:
      running = false;
      manualEnabled = false;
      Serial.println(F("DONE: at HOME, claw open; M enables joystick."));
      return;
  }
  Serial.print(F("STAGE ")); Serial.println(stage);
  beginMove(goal);
}
int interpolateAngle(int from, int to) {
  return from + (long)(to - from) * moveProgress / moveSteps;
}
void updateMotion() {
  if (!running) return;
  unsigned long now = millis();
  if (settling) {
    if (now - settleSince >= 300UL) { ++stage; beginStage(); }
    return;
  }
  if (moveProgress < moveSteps) {
    if (now - lastStep < (unsigned long)servoDelay) return;
    lastStep = now;
    ++moveProgress;
    current.b = interpolateAngle(startAngle.b, targetAngle.b);
    current.r = interpolateAngle(startAngle.r, targetAngle.r);
    current.f = interpolateAngle(startAngle.f, targetAngle.f);
    current.c = interpolateAngle(startAngle.c, targetAngle.c);
    writeCurrent();
  }
  if (moveProgress >= moveSteps) { settling = true; settleSince = now; }
}
void startObject(byte index) {
  ObjectPlan &p = plans[index];
  if (p.taught != 15) {
    Serial.println(F("ERROR: placeholder points. SET all four points first.")); return;
  }
  if (!validAngles(home.angle)) { Serial.println(F("ERROR: invalid HOME.")); return; }
  for (byte j=0; j<4; ++j) {
    if (!validAngles(p.point[j].angle)) { Serial.println(F("ERROR: invalid point.")); return; }
  }
  if (p.openAngle < CLAW_MIN || p.closeAngle > CLAW_MAX || p.openAngle >= p.closeAngle) {
    Serial.println(F("ERROR: require CLAW_MIN <= OPEN < CLOSE <= CLAW_MAX.")); return;
  }
  activeObject = index; stage = 0; manualEnabled = false; running = true;
  beginStage();
}
void setCalibration(char *object, char *name) {
  if (object && !strcmp(object,"HOME") && !name) {
    home = makePoint(current); Serial.print(F("SET HOME ")); printPoint(home); return;
  }
  if (!object || strlen(object)!=1 || object[0]<'A' || object[0]>'C' || !name) {
    Serial.println(F("ERROR: SET HOME or SET A PICK/PICKUP/PLACE/PLACEUP/OPEN/CLOSE")); return;
  }
  ObjectPlan &p = plans[object[0]-'A'];
  if (!strcmp(name,"OPEN")) p.openAngle = current.c;
  else if (!strcmp(name,"CLOSE")) p.closeAngle = current.c;
  else {
    int index = -1;
    if (!strcmp(name,"PICK")) index = PICK;
    else if (!strcmp(name,"PICKUP")) index = PICKUP;
    else if (!strcmp(name,"PLACE")) index = PLACE;
    else if (!strcmp(name,"PLACEUP")) index = PLACEUP;
    if (index < 0) { Serial.println(F("ERROR: unknown point name.")); return; }
    p.point[index] = makePoint(current);
    p.taught |= (1 << index);
  }
  Serial.println(F("SET OK (RAM only).")); printPoint(makePoint(current));
}
void handleCommand(char *line) {
  for (char *p=line; *p; ++p) *p = toupper((unsigned char)*p);
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *arg1 = strtok(NULL, " \t");
  char *arg2 = strtok(NULL, " \t");
  char *extra = strtok(NULL, " \t");
  if (extra || (strcmp(cmd,"SET") && arg1)) {
    Serial.println(F("ERROR: extra arguments.")); return;
  }
  if (!strcmp(cmd,"X")) { abortMotion(); return; }
  if (!strcmp(cmd,"POS")) { printPoint(makePoint(current)); return; }
  if (!strcmp(cmd,"LIVE")) { liveOutput = !liveOutput; return; }
  if (!strcmp(cmd,"?") || !strcmp(cmd,"TABLE")) { reportTable(); return; }
  if (!strcmp(cmd,"H")) { servoDelay = clampVal(servoDelay-5,5,100); return; }
  if (!strcmp(cmd,"L")) { servoDelay = clampVal(servoDelay+5,5,100); return; }
  if (running) { Serial.println(F("BUSY: X stops motion.")); return; }
  if (!strcmp(cmd,"M")) { manualEnabled=true; Serial.println(F("Joystick ON")); return; }
  if (!strcmp(cmd,"SET")) { setCalibration(arg1,arg2); return; }
  if (strlen(cmd)==1 && cmd[0]>='A' && cmd[0]<='C') { startObject(cmd[0]-'A'); return; }
  Serial.println(F("ERROR: A/B/C, X, M, POS, ?, LIVE, H/L, SET HOME, SET A PICK/PICKUP/PLACE/PLACEUP/OPEN/CLOSE"));
}
void readSerialLine() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch=='\r' || ch=='\n') {
      if (lineOverflow) Serial.println(F("ERROR: line too long; entire line discarded."));
      else if (lineLen) { lineBuf[lineLen]='\0'; handleCommand(lineBuf); }
      lineLen=0; lineOverflow=false;
    } else if (!lineOverflow) {
      if (lineLen < sizeof(lineBuf)-1) lineBuf[lineLen++]=ch;
      else lineOverflow=true;
    }
  }
}
void setup() {
  Serial.begin(9600);
  home = makePoint(HOME_ANGLES);
  for (byte i=0; i<3; ++i) {
    for (byte j=0; j<4; ++j) plans[i].point[j] = makePoint(PLACEHOLDERS[i][j]);
    plans[i].openAngle=OPEN_ANGLES[i]; plans[i].closeAngle=CLOSE_ANGLES[i];
    plans[i].taught=INITIAL_TAUGHT[i];
  }
  // 开机指令值与内部 current 一致，沿用 JoyFK_alpha2 的四个90度。
  writeCurrent();
  base.attach(PIN_BASE); rArm.attach(PIN_RARM);
  fArm.attach(PIN_FARM); claw.attach(PIN_CLAW);
  writeCurrent();
  Serial.println(F("Task2_alpha2 | 9600 baud | commands end with newline"));
  Serial.println(F("Joystick ON. SET A PICK/PICKUP/PLACE/PLACEUP; repeat B/C."));
  Serial.println(F("Defaults are placeholders. Calibrate grip OPEN/CLOSE and HOME. X stops; M resumes joystick."));
}
void loop() {
  readSerialLine();
  updateMotion();
  updateJoystick();
  if (liveOutput && millis()-lastPrint >= PRINT_MS) {
    lastPrint=millis(); printPoint(makePoint(current));
  }
}
