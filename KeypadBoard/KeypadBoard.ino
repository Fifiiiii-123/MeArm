/*
 * ============================================================================
 *  文件：KeypadBoard.ino
 *  工程：Task3 按键板程序（第二块板 · 单独烧录）
 * ----------------------------------------------------------------------------
 *  功能：读取 4 个按键，按下后通过串口发送指令给主控板（Task3.ino）
 *        主控板收到后驱动机械臂：
 *          '1' 循环夹取（A→B→C→A…）
 *          '2' 录制开始/结束（摇杆操控 >10s）
 *          '3' 播放上一次录制
 *          '4' 回中
 *
 *  接线（第二块板，Arduino UNO R3）：
 *    按键1 → D8         按键2 → D9
 *    按键3 → D10        按键4 → D11
 *    （按键另一端接 GND，按下=低电平；代码已用内部上拉，无需外部电阻）
 *
 *  与主控板连接（两板波特率均 9600）：
 *    本板 TX(D1) → 主控板 RX(D0)
 *    本板 GND    → 主控板 GND
 *
 *  注意：主控板舵机引脚（D9/D8/D7/D6）保持不变，与按键互不冲突。
 * ============================================================================
 */

#define KEY1_PIN  8           // 按键1：循环夹取
#define KEY2_PIN  9           // 按键2：录制/结束
#define KEY3_PIN  10          // 按键3：播放
#define KEY4_PIN  11          // 按键4：回中

const unsigned long DEBOUNCE_MS = 300;   // 防抖时间（毫秒）

unsigned long lastPress1 = 0;   // 各按键上次触发时间（防抖用）
unsigned long lastPress2 = 0;
unsigned long lastPress3 = 0;
unsigned long lastPress4 = 0;

void setup(){
  pinMode(KEY1_PIN, INPUT_PULLUP);   // 按下=低电平，内部上拉
  pinMode(KEY2_PIN, INPUT_PULLUP);
  pinMode(KEY3_PIN, INPUT_PULLUP);
  pinMode(KEY4_PIN, INPUT_PULLUP);

  Serial.begin(9600);                // 与主控板同波特率
}

void loop(){
  unsigned long now = millis();

  if (digitalRead(KEY1_PIN) == LOW && now - lastPress1 > DEBOUNCE_MS){
    lastPress1 = now;
    Serial.println('1');      // 循环夹取
  }
  if (digitalRead(KEY2_PIN) == LOW && now - lastPress2 > DEBOUNCE_MS){
    lastPress2 = now;
    Serial.println('2');      // 录制/结束
  }
  if (digitalRead(KEY3_PIN) == LOW && now - lastPress3 > DEBOUNCE_MS){
    lastPress3 = now;
    Serial.println('3');      // 播放
  }
  if (digitalRead(KEY4_PIN) == LOW && now - lastPress4 > DEBOUNCE_MS){
    lastPress4 = now;
    Serial.println('4');      // 回中
  }
}
