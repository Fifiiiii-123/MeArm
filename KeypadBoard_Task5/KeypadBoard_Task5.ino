/*
 * ============================================================================
 *  文件：KeypadBoard_Task5.ino
 *  工程：KeypadBoard_Task5 —— 任务五按键板（第二块 UNO R3）
 * ----------------------------------------------------------------------------
 *  功能：读取 4 个按键（D8/D9/D10/D11，按下=低电平），识别单击/双击/三击，
 *        把对应指令字符通过串口（9600）发送给主控板 Task5.ino。
 *
 *  按键映射（与 Task5.ino 的 handleCommand 一致）：
 *    键1 D8 ：单击 → '1' 示教/记录点
 *    键2 D9 ：单击 → '2' 暂停       双击 → '3' 继续
 *    键3 D10：单击 → '4' 取消       双击 → '5' 折线绘制    三击 → '6' 曲线绘制
 *    键4 D11：单击 → '7' 直线绘制   双击 → '8' 基础图形
 *
 *  连击判定：按键释放后 350ms 内再次按下计为下一击；
 *            释放后 350ms 内无新按下，则按本次总击数发送指令。
 *  接线：按键一端接对应引脚，另一端接 GND（靠引脚内部上拉 INPUT_PULLUP）。
 * ============================================================================
 */
#define K1 8                // 键1：示教/记录
#define K2 9                // 键2：暂停 / 继续
#define K3 10               // 键3：取消 / 折线 / 曲线
#define K4 11               // 键4：直线 / 图形

#define DEBOUNCE_MS  30     // 按键消抖时间
#define CLICK_WINDOW 350    // 连击判定窗口（释放后等待，ms）

/* ---------------- 按键状态机 ----------------
 * 每个键独立维护：
 *   stable   防抖确认后的当前电平（true=松开，false=按下）
 *   prev     上一次读取的原始电平（用于检测变化）
 *   clicks   本次连续按下的次数（单击=1，双击=2，三击=3）
 *   settle   是否在等待结算（有击数但尚未超窗发送）
 */
struct Key {
  uint8_t  pin;
  uint8_t  clicks;
  bool     stable;
  bool     prev;
  uint32_t debounceAt;
  uint32_t releaseAt;
  bool     settle;
};

Key keys[4] = {
  { K1, 0, true, true, 0, 0, false },
  { K2, 0, true, true, 0, 0, false },
  { K3, 0, true, true, 0, 0, false },
  { K4, 0, true, true, 0, 0, false },
};

/* ==================== 初始化 ==================== */
void setup(){
  Serial.begin(9600);
  pinMode(K1, INPUT_PULLUP);                 // 按下=低电平
  pinMode(K2, INPUT_PULLUP);
  pinMode(K3, INPUT_PULLUP);
  pinMode(K4, INPUT_PULLUP);
  Serial.println("=== Task5 按键板就绪（D8~D11）===");
}

/* ==================== 按键映射 ====================
 * 按引脚与击数查表发送指令字符；无对应映射则忽略 */
void sendCommand(uint8_t pin, uint8_t n){
  char cmd = 0;
  if (pin == K1 && n == 1)       cmd = '1';   // 示教/记录
  else if (pin == K2 && n == 1)  cmd = '2';   // 暂停
  else if (pin == K2 && n == 2)  cmd = '3';   // 继续
  else if (pin == K3 && n == 1)  cmd = '4';   // 取消
  else if (pin == K3 && n == 2)  cmd = '5';   // 折线绘制
  else if (pin == K3 && n == 3)  cmd = '6';   // 曲线绘制
  else if (pin == K4 && n == 1)  cmd = '7';   // 直线绘制
  else if (pin == K4 && n == 2)  cmd = '8';   // 基础图形
  if (cmd){
    Serial.println(cmd);                      // 发送（换行结尾，主控按行解析）
  }
}

/* ==================== 单键扫描 ====================
 * 消抖 → 计击数 → 释放后超窗结算 */
void processKey(Key &k, uint32_t now){
  bool raw = digitalRead(k.pin);

  /* 电平变化：记录防抖起点 */
  if (raw != k.prev){
    k.prev = raw;
    k.debounceAt = now;
    return;
  }

  /* 稳定超过 DEBOUNCE_MS 才确认电平变化 */
  if (raw != k.stable && now - k.debounceAt >= DEBOUNCE_MS){
    k.stable = raw;
    if (raw == LOW){                          /* 确认按下 */
      k.clicks++;                             // 击数 +1
      k.settle = true;                        // 进入结算等待
    } else {                                  /* 确认释放 */
      k.releaseAt = now;                      // 结算窗口从释放起算
    }
  }

  /* 结算：已释放且 CLICK_WINDOW 内无新按下 */
  if (k.settle && k.stable && now - k.releaseAt >= CLICK_WINDOW){
    sendCommand(k.pin, k.clicks);
    k.clicks = 0;
    k.settle = false;
  }
}

/* ==================== 主循环 ==================== */
void loop(){
  uint32_t now = millis();
  for (int i = 0; i < 4; i++){
    processKey(keys[i], now);
  }
}
