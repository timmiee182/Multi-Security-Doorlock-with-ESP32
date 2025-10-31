#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <MFRC522.h>
#include <Adafruit_Fingerprint.h>
#include <LiquidCrystal_I2C.h>
#include <esp_task_wdt.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define BLYNK_TEMPLATE_ID "xxxxx" //use your own 
#define BLYNK_TEMPLATE_NAME "Smart Door"
#define BLYNK_AUTH_TOKEN "xxxxxxxx"
#define BLYNK_PRINT Serial

// ==================== BLYNK ANF WI-FI CONF ====================
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
//USE YOUR OWN CREDENTIALS
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "xxxxxxx";
char pass[] = "xxxxxxx";

// add obj for NTP client
WiFiUDP ntpUDP;
//Timezone GMT+7 (7 * 3600 secs)
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600);

// DEFINE Virtual Pin for setting up on Blynk APP
#define VPIN_LOCK_STATUS   V0
#define VPIN_LOG           V1 
#define VPIN_REMOTE_UNLOCK V2
#define VPIN_DUMP_LOG      V3 

BlynkTimer timer;
// declare connection func
void checkBlynkConnection();
// =========================================================================

// Declare Preferences for Logging and how much Logs can be held
Preferences prefs_log;
const int MAX_LOG_ENTRIES = 50; // the first 50 logs will be saved here

//==================== RELAY + LED + BUZZER ====================
#define RELAY_PIN   34    // RELAY signal (Active HIGH)+Green LED
#define RED_LED     26   // RED LED 
#define BUZZER_PIN  27   // BUZZER

//==================== LCD I2C SDA=21 SCl=22 ====================
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

//==================== KEYPAD 4x4 qua PCF8574 SDA=21 SCl=22 ====================
#define PCF_KEYPAD_ADDR 0x20
const char keymap[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
const uint8_t ROW_PINS[4] = {0,1,2,3};
const uint8_t COL_PINS[4] = {4,5,6,7};

//==================== RFID RC522 (SPI) ====================
#define SS_PIN    5
#define RST_PIN   4
#define SCK_PIN   18
#define MOSI_PIN  23
#define MISO_PIN  19
MFRC522 mfrc522(SS_PIN, RST_PIN);

//==================== adafruit fingerprint  (UART2) ====================
#define FINGERPRINT_RX 16
#define FINGERPRINT_TX 17
HardwareSerial mySerial(2);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

//==================== System Conf ====================
const int MAX_LEN = 4;
unsigned long UNLOCK_TIME_MS = 3000; // Unlock in 3 Secs
unsigned long lastFingerScanMs = 0;
unsigned long lastRfidScanMs = 0;
const unsigned long SENSOR_SCAN_INTERVAL = 250; // Scan the sensor in every 250ms
//NVS
Preferences prefs_rfid;        // namespace: "rfid"  (UIDs)
Preferences prefs_cfg;         // namespace: "cfg"   (MASTER_PIN)
String MASTER_PIN = "1234";    // default (read from NVS when booting up)
const int MAX_UIDS = 50;       // holds upto 50 RFID Cards

//==================== Satate Variables ====================
enum SystemState {
  STATE_NORMAL,
  STATE_ADMIN_LOGIN,
  STATE_ADMIN_MENU,
  STATE_CHANGE_PASS,
  STATE_MANAGE_RFID_MENU,
  STATE_RFID_ADD,
  STATE_RFID_DELETE,
  STATE_MANAGE_FINGER_MENU,
  STATE_FINGER_ADD,
  STATE_FINGER_DELETE
};
SystemState currentState = STATE_NORMAL;

String inputBuffer = "";
bool keyIsDown = false;
int lastStableRaw = -1;
bool adminArmed = false; 
bool isUnlocked = false;
unsigned long unlockDeadline = 0;
int failCount = 0; 

//==================== LCD auto-off ====================
unsigned long lastActionMs = 0;
bool lcdIsOn = true;
const unsigned long LCD_TIMEOUT = 15000; // 15s

// --- PIN input timeout ---
unsigned long pinInputStartMs = 0; 
const unsigned long PIN_INPUT_TIMEOUT = 10000;     // 10s

//==================== declares functions ====================
void showHomeScreen();
void displayMessage(String line1, String line2, int delay_ms);
void displayAdminMenu();
void displayManageRFIDMenu();
void displayManageFingerMenu();
void showMaskedInput();
void handleKeypad();
void processKeyPress(char key);
void handleConfirmPress();
void unlockDoor(String msg, bool isRfid = false); 
void lockDoor();
void wakeLCD();
void manage_RFID_add();
void saveRfidLog(String uid);

static inline void pcfWrite(uint8_t b) { Wire.beginTransmission(PCF_KEYPAD_ADDR); Wire.write(b); Wire.endTransmission(); }
static inline uint8_t pcfRead() { Wire.requestFrom((int)PCF_KEYPAD_ADDR, 1); return Wire.available() ? Wire.read() : 0xFF; }
int scanKeyRaw();
char rawToChar(int raw);

String readCard();
bool isUIDStored(const String &uid);
void addUID(const String &uid);
void deleteUIDByIndex(int index);
String uidToHexString(byte *uid, byte len);

uint8_t getFingerprintEnroll(uint8_t id);
int getFingerprintIDez();
uint8_t deleteFingerprint(uint8_t id);

// ===== Helpers LED/Buzzer/Alarm =====
void beepShort(int ms = 200) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}
void indicateFailOnce() {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);
  delay(250);
  digitalWrite(RED_LED, LOW);
}
void triggerAlarm10s() {
  Serial.println("Sai 3 lan! Bao dong 10s!");
// Sends the alarm to your device thru Blynk account
  if (Blynk.connected()) {
    // "pin_fail_alarm" event code for blynk
    Blynk.logEvent("pin_fail_alarm", "Canh bao: Nhap sai PIN 3 lan!"); 
  }
  lcd.clear(); lcd.print("Bao dong 10s!");
  digitalWrite(RED_LED, HIGH);
  unsigned long t0 = millis();
  while (millis() - t0 < 10000) {
    Blynk.run(); 
    digitalWrite(BUZZER_PIN, HIGH);
    esp_task_wdt_reset();
    handleKeypad(); 
    delay(10);
  }
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RED_LED, LOW);
  failCount = 0; 
  showHomeScreen();
}

//==================== Setup ====================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // ==================== Blynk Start-ups ====================
  lcd.print("He thong khoi dong");
  lcd.setCursor(0, 1);
  lcd.print("Dang ket noi WiFi...");
  
  WiFi.begin(ssid, pass);
  Blynk.config(auth); 
  timer.setInterval(5000L, checkBlynkConnection);
  // =================================================================
// Initialize NTP client
    timeClient.begin();
  // --- Watchdog set-up---
  const esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,   // 10s
      .idle_core_mask = 0,
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  pcfWrite(0xFF);

  // RFID
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  mfrc522.PCD_Init();
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max); 
  SPI.setFrequency(1000000); 

  // NVS
  prefs_rfid.begin("rfid", false);
  prefs_cfg.begin("cfg",   false);
  prefs_log.begin("log_unlock", false);
  MASTER_PIN = prefs_cfg.getString("masterPIN", "1234");

  // Fingerprint
  mySerial.begin(57600, SERIAL_8N1, FINGERPRINT_RX, FINGERPRINT_TX);
  finger.begin(57600);
  if (!finger.verifyPassword()) {
    Serial.println("WARNING: Khong tim thay cam bien van tay!");
    lcd.clear(); lcd.print("Loi cam bien VT");
    delay(900);
  }

  showHomeScreen();
  lastActionMs = millis();
  lcdIsOn = true;
  
  // ==================== Displays WI-FI State====================
  int wifi_retries = 10;
  while(WiFi.status() != WL_CONNECTED && wifi_retries > 0) {
    delay(500);
    wifi_retries--;
  }

  if(WiFi.status() == WL_CONNECTED) {
    displayMessage("WiFi Connected", "", 500);
  } else {
    displayMessage("WiFi Failed", "Offline Mode", 500);
  }
}

//==================== Loop ====================
void loop() {
  Blynk.run();
  timer.run();
  esp_task_wdt_reset();

  // Deactivate the latch timeout
  if (isUnlocked && millis() > unlockDeadline) {
    lockDoor();
  }

  // Chỉ quét Vân tay và RFID khi đang ở màn hình chính
  if (currentState == STATE_NORMAL) {
    // 1) Vân tay (Non-blocking)
    if (millis() - lastFingerScanMs > SENSOR_SCAN_INTERVAL) {
      lastFingerScanMs = millis(); // Đặt lại mốc thời gian
      int fingerID = getFingerprintIDez();
      if (fingerID > 0) {
        wakeLCD();
        Serial.printf("Fingerprint OK (ID=%d)\n", fingerID);
        unlockDoor("Van tay", false);
        saveUnlockLog("Van tay");
        String fingerprintLog = "Van tay ID: " + String(fingerID);
        saveUnlockLog(fingerprintLog); // Lưu log có ID
        adminArmed = false;
      } else if (fingerID == 0) { // Không khớp
        wakeLCD();
        Serial.println("Fingerprint not matched!");
        displayMessage("Van tay khong hop le", "", 600);
        failCount++;
        indicateFailOnce();
        if (failCount >= 5) { triggerAlarm10s(); }
      }
      // Bỏ qua trường hợp fingerID < 0 (lỗi đọc/không có tay) để tránh báo động nhầm
    } // Kết thúc if check thời gian quét vân tay

    // 2) RFID (Non-blocking)
    if (millis() - lastRfidScanMs > SENSOR_SCAN_INTERVAL + 50) { // Tránh quét cùng lúc với vân tay
      lastRfidScanMs = millis(); // Đặt lại mốc thời gian
      String uid = readCard();
      if (uid != "") {
        if (isUIDStored(uid)) { // Thẻ hợp lệ
          wakeLCD();
          Serial.printf("RFID OK: %s\n", uid.c_str());
          unlockDoor(uid, true);
          adminArmed = false;
        } else { // Thẻ không hợp lệ
          displayMessage("The khong hop le", "", 800);
          indicateFailOnce();
        }
      }
    } // Kết thúc if check thời gian quét RFID

  } // <<<<< Dấu ngoặc đóng của if (currentState == STATE_NORMAL)

  // Luôn xử lý bàn phím ở bất kỳ trạng thái nào
  handleKeypad();

  // Luôn kiểm tra kết nối I2C Keypad
  Wire.beginTransmission(PCF_KEYPAD_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("⚠️ I2C keypad not responding, reinit bus...");
    Wire.begin();
    pcfWrite(0xFF);
  }

  // Luôn kiểm tra timeout để tắt LCD
  if (lcdIsOn && millis() - lastActionMs > LCD_TIMEOUT) {
    if (currentState != STATE_NORMAL) { // Nếu đang ở menu admin -> về home rồi tắt
      showHomeScreen();
      lcd.noBacklight();
      lcdIsOn = false;
    } else { // Nếu đang ở home -> tắt luôn
      lcd.noBacklight();
      lcdIsOn = false;
    }
  }
} // <<<<< Dấu ngoặc đóng của hàm loop()

//==================== LCD helpers ====================
void showHomeScreen() {
  inputBuffer = "";
  currentState = STATE_NORMAL;
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Xin moi xac thuc");
  lcd.setCursor(0,1); lcd.print("PIN:");
  wakeLCD();
}

void wakeLCD() {
  lastActionMs = millis();
  if (!lcdIsOn) {
    lcd.backlight();
    lcdIsOn = true;
  }
}

void displayMessage(String line1, String line2, int delay_ms) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(line1);
  lcd.setCursor(0,1); lcd.print(line2);
  wakeLCD();

  unsigned long start = millis();
  while (millis() - start < (unsigned long)delay_ms) {
    Blynk.run(); 
    handleKeypad();  
    esp_task_wdt_reset();
    delay(5);
  }

  if (!isUnlocked && currentState == STATE_NORMAL) showHomeScreen();
}

void showMaskedInput() {
  const int startCol = 5;
  lcd.setCursor(startCol, 1);
  String masked = "";
  for (size_t i = 0; i < inputBuffer.length(); i++) masked += "*";
  lcd.print(masked);
  int remain = 16 - startCol - masked.length();
  while (remain-- > 0) lcd.print(' ');
}


//==================== Door control ====================
void unlockDoor(String msg, bool isRfid) {
  displayMessage("Xac thuc OK!", "", 250); 

  if (Blynk.connected()) {
    Blynk.virtualWrite(VPIN_LOCK_STATUS, 1); 
    String logMessageBase = msg; // Mặc định chỉ gửi phương thức/UID
// Chỉ định dạng lại nếu là RFID
    if (isRfid) {
      logMessageBase = "RFID UID: " + msg; // Ghép thêm "RFID UID: "
    }
// --- Phần lấy và thêm timestamp ---
    String liveLog = logMessageBase;

    // Cố gắng lấy và thêm timestamp nếu NTP đã đồng bộ
    timeClient.update();
    if (timeClient.isTimeSet()) {
        int currentHour = timeClient.getHours();
        int currentMinute = timeClient.getMinutes();

        unsigned long utcEpochTime = timeClient.getEpochTime(); // Lấy epoch time UTC
        time_t localEpochTime = utcEpochTime + (7 * 3600); // Cộng 7 tiếng
        struct tm *ptm = gmtime(&localEpochTime);

        int currentDay = ptm->tm_mday;
        int currentMonth = ptm->tm_mon + 1;
        int currentYear = ptm->tm_year + 1900;

        String dateStamp = "";
        if (currentDay < 10) dateStamp += "0";
        dateStamp += String(currentDay) + "-";
        if (currentMonth < 10) dateStamp += "0";
        dateStamp += String(currentMonth) + "-";
        dateStamp += String(currentYear);

        String timeStamp = "";
        if (currentHour < 10) timeStamp += "0";
        timeStamp += String(currentHour) + ":";
        if (currentMinute < 10) timeStamp += "0";
        timeStamp += String(currentMinute);

        liveLog = dateStamp + " " + timeStamp + " " + msg; // Ghép thêm ngày giờ
    } else {
        Serial.println("CẢNH BÁO: Chưa lấy được giờ NTP, gửi log không có timestamp.");
    }
    Blynk.virtualWrite(VPIN_LOG, liveLog + "\n");

    if (isRfid) {
      Blynk.logEvent("rfid_unlock"); 
      saveUnlockLog(logMessageBase); // <-- [MỚI] Lưu UID vào Log
    }
  }

  digitalWrite(RELAY_PIN, HIGH);       
  digitalWrite(GREEN_LED, HIGH);       
  isUnlocked = true;
  unlockDeadline = millis() + UNLOCK_TIME_MS;
  displayMessage("CUA DA MO", "Xin moi vao", UNLOCK_TIME_MS - 150);
  digitalWrite(GREEN_LED, LOW);
  failCount = 0;
}

void lockDoor() {
  digitalWrite(RELAY_PIN, LOW);        
  isUnlocked = false;
  Serial.println("Door locked");
  showHomeScreen();
  
  if (Blynk.connected()) {
    Blynk.virtualWrite(VPIN_LOCK_STATUS, 0); 
  }
  // ==================== [THE FIX] ====================
  // Proactively reset the RFID module, as the relay
  // can cause it to hang.
  Serial.println("Re-initializing RFID module...");
  mfrc522.PCD_Init();
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max); 
  // ===================================================
}

//==================== Keypad (Giữ nguyên) ====================
int scanKeyRaw() {
  uint8_t val = 0xFF;
  for (uint8_t r = 0; r < 4; r++) {
    val = 0xFF & ~(1 << ROW_PINS[r]);
    pcfWrite(val);
    delayMicroseconds(100);  
    uint8_t in = pcfRead();
    for (uint8_t c = 0; c < 4; c++) {
      if (!(in & (1 << COL_PINS[c]))) {
        pcfWrite(0xFF);
        delayMicroseconds(100);
        return r * 4 + c;
      }
    }
  }
  pcfWrite(0xFF);
  return -1;
}

char rawToChar(int raw) {
  if (raw < 0) return 0;
  return keymap[raw / 4][raw % 4];
}

void handleKeypad() {
  static unsigned long lastChangeMs = 0;
  static int currRaw = -1;
  int raw = scanKeyRaw();
  if (raw != currRaw) {
    currRaw = raw;
    lastChangeMs = millis();
  }
  if (millis() - lastChangeMs > 70) {
    if (!keyIsDown && currRaw >= 0) {
      keyIsDown = true;
      lastStableRaw = currRaw;
      char k = rawToChar(lastStableRaw);
      processKeyPress(k);
    } else if (keyIsDown && currRaw < 0) {
      keyIsDown = false;
      lastStableRaw = -1;
    }
  }
  if ((currentState == STATE_NORMAL || currentState == STATE_ADMIN_LOGIN) &&
      inputBuffer.length() > 0 &&
      millis() - pinInputStartMs > PIN_INPUT_TIMEOUT) {
    inputBuffer = "";
    showHomeScreen();
  }
}

void processKeyPress(char key) {
  wakeLCD();
  Serial.print("Key: "); Serial.println(key);

  if (key == '*') {
    if ((currentState == STATE_FINGER_ADD || currentState == STATE_FINGER_DELETE) && inputBuffer.length() > 0) {
      inputBuffer.remove(inputBuffer.length() - 1);
      lcd.setCursor(0, 1);
      lcd.print("                ");  
      lcd.setCursor(0, 1);
      lcd.print(inputBuffer);        
      return;
    }
    if (currentState == STATE_NORMAL) {
      adminArmed = true;
      lcd.setCursor(0,0); lcd.print("Nhan # de vao  ");
      lcd.setCursor(0,1); lcd.print("Che do Admin    ");
    } else if (currentState != STATE_ADMIN_LOGIN) {
      if (currentState == STATE_MANAGE_RFID_MENU || currentState == STATE_MANAGE_FINGER_MENU || currentState == STATE_CHANGE_PASS) {
        currentState = STATE_ADMIN_MENU;
        displayAdminMenu();
      } else if (currentState == STATE_RFID_ADD || currentState == STATE_RFID_DELETE) {
        currentState = STATE_MANAGE_RFID_MENU;
        displayManageRFIDMenu();
      } else if (currentState == STATE_FINGER_ADD || currentState == STATE_FINGER_DELETE) {
        currentState = STATE_MANAGE_FINGER_MENU;
        displayManageFingerMenu();
      } else {
        showHomeScreen();
      }
    }
    return;
  }

  if (key == '#') {
    if (adminArmed && currentState == STATE_NORMAL) {
      adminArmed = false;
      currentState = STATE_ADMIN_LOGIN;
      inputBuffer = "";
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Che do Admin");
      lcd.setCursor(0,1); lcd.print("PIN: ");
      return;
    }
    handleConfirmPress();
    return;
  }

  if (key >= '0' && key <= '9') {
    if (currentState == STATE_FINGER_ADD || currentState == STATE_FINGER_DELETE) {
      if (inputBuffer.length() < 3) {        
        inputBuffer += key;
        lcd.setCursor(0,1);
        lcd.print("                ");        
        lcd.setCursor(0,1);
        lcd.print(inputBuffer);              
      }
      return;
    }
    if (currentState == STATE_NORMAL || currentState == STATE_ADMIN_LOGIN || currentState == STATE_CHANGE_PASS) {
      if (inputBuffer.length() == 0) pinInputStartMs = millis();
      if ((int)inputBuffer.length() < MAX_LEN) {
        inputBuffer += key;
        showMaskedInput();
        pinInputStartMs = millis();
        if (currentState == STATE_NORMAL && inputBuffer.length() == MAX_LEN) {
          if (inputBuffer.equals(MASTER_PIN)) {
            unlockDoor("PIN", false);
            saveUnlockLog("PIN"); 
          } else {
            failCount++; // <-- Moved up
            String triesLeftMsg = "Con lai " + String(3 - failCount) + " lan";
            displayMessage("Sai PIN!", triesLeftMsg, 600); // <-- Changed
            indicateFailOnce();
            if (failCount >= 3) { triggerAlarm10s(); }
          }
          inputBuffer = "";
        } else if (currentState == STATE_ADMIN_LOGIN && inputBuffer.length() == MAX_LEN) {
          if (inputBuffer.equals(MASTER_PIN)) {
            failCount = 0; // <-- ADD THIS to reset on success
            currentState = STATE_ADMIN_MENU;
            displayAdminMenu();
          } else {
            failCount++; // <-- Moved up
            String triesLeftMsg = "Con lai " + String(3 - failCount) + " lan";
            displayMessage("Sai PIN!", triesLeftMsg, 600); // <-- Changed
            indicateFailOnce();
            if (failCount >= 3) { triggerAlarm10s(); }
            showHomeScreen();
          }
          inputBuffer = "";
        }
      }
      return; 
    }
  }

  if (currentState == STATE_ADMIN_MENU) {
    if (key == '1') {
      currentState = STATE_MANAGE_FINGER_MENU;
      displayManageFingerMenu();
    } else if (key == '2') {
      currentState = STATE_MANAGE_RFID_MENU;
      displayManageRFIDMenu();
    } else if (key == '3') {
      currentState = STATE_CHANGE_PASS;
      inputBuffer = "";
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Doi PIN");
      lcd.setCursor(0,1); lcd.print("PIN: ");
    }
    return;
  }

  if (currentState == STATE_MANAGE_RFID_MENU) {
    if (key == '1') {
      currentState = STATE_RFID_ADD;
      manage_RFID_add();
    } else if (key == '2') {
      currentState = STATE_RFID_DELETE;
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Xoa The RFID");
      lcd.setCursor(0,1); lcd.print("STT: ");
      inputBuffer = "";
    }
    return;
  }

  if (currentState == STATE_MANAGE_FINGER_MENU) {
    if (key == '1') {
      currentState = STATE_FINGER_ADD;
      inputBuffer = "";
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Them Van Tay(ID)");
      lcd.setCursor(0,1); lcd.print("                "); 
      return;
    } else if (key == '2') {
      currentState = STATE_FINGER_DELETE;
      inputBuffer = "";
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Xoa Van Tay(ID)");
      lcd.setCursor(0,1); lcd.print("                "); 
      return;
    }
  }

  inputBuffer += key;
  showMaskedInput();
}

//==================== Admin Menus (Giữ nguyên) ====================
void displayAdminMenu() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("MENU ADMIN");
  lcd.setCursor(0,1); lcd.print("1.VT 2.RFID 3.PIN");
}

void displayManageRFIDMenu() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("QL The RFID");
  lcd.setCursor(0,1); lcd.print("1.Them  2.Xoa");
}

void displayManageFingerMenu() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("QL Van Tay");
  lcd.setCursor(0,1); lcd.print("1.Them  2.Xoa");
}

//==================== RFID helpers (Giữ nguyên) ====================
String uidToHexString(byte *uid, byte len) {
  String s = "";
  for (byte i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
  }
  s.toUpperCase();
  return s;
}

void handleConfirmPress() {
  wakeLCD();
  switch (currentState) {
    case STATE_NORMAL: {
      if (inputBuffer.equals(MASTER_PIN)) {
        unlockDoor("PIN",false);
        saveUnlockLog("PIN");
      } else {
        failCount++; // <-- Moved up
        String triesLeftMsg = "Con lai " + String(3 - failCount) + " lan";
        displayMessage("Sai PIN!", triesLeftMsg, 600); // <-- Changed
        indicateFailOnce();
        if (failCount >= 3) { triggerAlarm10s(); }
      }
      break;
    }
    case STATE_ADMIN_LOGIN: {
      if (inputBuffer.equals(MASTER_PIN)) {
        failCount = 0; // <-- ADD THIS to reset on success
        currentState = STATE_ADMIN_MENU;
        displayAdminMenu();
      } else {
        failCount++; // <-- Moved up
        String triesLeftMsg = "Con lai " + String(3 - failCount) + " lan";
        displayMessage("Sai PIN!", triesLeftMsg, 600); // <-- Changed
        indicateFailOnce();
        if (failCount >= 3) { triggerAlarm10s(); }
        showHomeScreen();
      }
      break;
    }
    case STATE_CHANGE_PASS: {
      if ((int)inputBuffer.length() == MAX_LEN) {
        MASTER_PIN = inputBuffer;
        prefs_cfg.putString("masterPIN", MASTER_PIN);
        displayMessage("Doi PIN OK", "", 600);
        currentState = STATE_ADMIN_MENU;
        displayAdminMenu();
      } else {
        displayMessage("PIN phai 4 so", "", 600);
      }
      break;
    }
    case STATE_RFID_DELETE: {
      int idx = inputBuffer.toInt();
      uint16_t count = prefs_rfid.getUShort("count", 0);
      if (idx < 0 || idx >= count) {
        displayMessage("STT khong hop le!", "", 800);
      } else {
        deleteUIDByIndex(idx);
        displayMessage("Xoa thanh cong!", "STT: " + String(idx), 800);
      }
      currentState = STATE_MANAGE_RFID_MENU;
      displayManageRFIDMenu();
      break;
    }
    case STATE_FINGER_ADD: {
      int id = inputBuffer.toInt();
      if (id < 1 || id > 127) {
        displayMessage("ID khong hop le", "Thu lai (1-127)", 800);
      } else {
        displayMessage("ID: " + String(id), "Dat ngon tay...", 0);
        getFingerprintEnroll(id);
      }
      currentState = STATE_MANAGE_FINGER_MENU;
      displayManageFingerMenu();
      break;
    }
    case STATE_FINGER_DELETE: {
      int id = inputBuffer.toInt();
      if (id < 1 || id > 127) {
        displayMessage("ID khong hop le", "Thu lai (1-127)", 800);
      } else {
        uint8_t p = finger.loadModel(id);
        if (p == FINGERPRINT_OK) {
          p = finger.deleteModel(id);
          if (p == FINGERPRINT_OK) {
            displayMessage("Xoa thanh cong", "ID: " + String(id), 800);
          } else {
            displayMessage("Xoa that bai", "Loi giao tiep/flash", 900);
          }
        } else {
          displayMessage("Xoa that bai", "ID khong ton tai", 900);
        }
      }
      currentState = STATE_MANAGE_FINGER_MENU;
      displayManageFingerMenu();
      break;
    }
    default:
      break;
  }
  inputBuffer = "";
} 

void resetRFID() {
  Serial.println("⚠️  Reinit RC522...");
  mfrc522.PCD_Reset();
  mfrc522.PCD_Init();
  delay(50);
}

String readCard() {
  static int readFailCount = 0;
  if (!mfrc522.PICC_IsNewCardPresent()) return "";
  if (!mfrc522.PICC_ReadCardSerial()) {
    if (++readFailCount >= 3) { resetRFID(); readFailCount = 0; }
    return "";
  }
  readFailCount = 0;
  String hexUID = uidToHexString(mfrc522.uid.uidByte, mfrc522.uid.size);
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(3);  
  return hexUID;
}

bool isUIDStored(const String &uid) {
  uint16_t count = prefs_rfid.getUShort("count", 0);
  for (uint16_t i = 0; i < count; i++) {
    String key = "u" + String(i);
    if (prefs_rfid.getString(key.c_str(), "").equalsIgnoreCase(uid)) return true;
  }
  return false;
}

void addUID(const String &uid) {
  if (isUIDStored(uid)) return;
  uint16_t count = prefs_rfid.getUShort("count", 0);
  if (count >= MAX_UIDS) return;
  String key = "u" + String(count);
  prefs_rfid.putString(key.c_str(), uid);
  prefs_rfid.putUShort("count", count + 1);
  Serial.println("Added UID: " + uid);
}

void deleteUIDByIndex(int index) {
  uint16_t count = prefs_rfid.getUShort("count", 0);
  if (index < 0 || index >= count) return;
  for (uint16_t j = index; j < count - 1; j++) {
    String curK = "u" + String(j);
    String nxtK = "u" + String(j + 1);
    prefs_rfid.putString(curK.c_str(), prefs_rfid.getString(nxtK.c_str(), ""));
  }
  prefs_rfid.remove(("u" + String(count - 1)).c_str());
  prefs_rfid.putUShort("count", count - 1);
  Serial.println("Deleted UID at index: " + String(index));
}

//==================== Fingerprint helpers ====================
uint8_t getFingerprintEnroll(uint8_t id) {
  int p = -1;
  unsigned long start = millis();
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    Blynk.run();
    esp_task_wdt_reset();
    if (millis() - start > 10000) {
      displayMessage("Het thoi gian", "", 800);
      return FINGERPRINT_TIMEOUT;
    }
    if (p == FINGERPRINT_NOFINGER) delay(100);
    else if (p != FINGERPRINT_OK) { displayMessage("Doc anh loi","Thu lai...", 600); }
  }
  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) { displayMessage("Xu ly loi","Thu lai...", 800); return p; }

  displayMessage("Nha ngon tay ra","...", 700);
  do { 
    p = finger.getImage(); 
    Blynk.run();
    esp_task_wdt_reset();
  } while (p != FINGERPRINT_NOFINGER);

  displayMessage("Dat lai ngon tay","cung vi tri...", 600);
  p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    Blynk.run();
    esp_task_wdt_reset();
    if (p == FINGERPRINT_NOFINGER) delay(100);
    else if (p != FINGERPRINT_OK) { displayMessage("Doc anh loi","Thu lai...", 600); }
  }
  p = finger.image2Tz(2);
  if (p != FINGERPRINT_OK) { displayMessage("Xu ly loi","Thu lai...", 800); return p; }

  p = finger.createModel();
  if (p != FINGERPRINT_OK) { displayMessage("Van tay khong khop","Thu lai tu dau", 1000); return p; }

  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    displayMessage("Them thanh cong!","ID: " + String(id), 900);
  } else {
    displayMessage("Loi luu tru","", 800);
  }
  return p;
}

int getFingerprintIDez() {
  int p = finger.getImage();
  if (p == FINGERPRINT_NOFINGER) return -2;        
  if (p != FINGERPRINT_OK) return -1;
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return -1;
  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) return 0;                
  return finger.fingerID;                           
}

//==================== Quản lý RFID: Thêm/Xóa ====================
void manage_RFID_add() {
  displayMessage("Them The RFID", "Quet the...", 0);
  esp_task_wdt_delete(NULL); 
  unsigned long start = millis();
  while (millis() - start < 8000) {
    Blynk.run();
    String uid = readCard();
    if (uid != "") {
      if (isUIDStored(uid)) {
        displayMessage("The da ton tai", "", 900);
      } else {
        addUID(uid);
        displayMessage("Them thanh cong", "UID: " + uid.substring(0,8), 1000);
      }
      esp_task_wdt_add(NULL); 
      currentState = STATE_MANAGE_RFID_MENU;
      displayManageRFIDMenu();
      return;
    }
    handleKeypad();
    delay(5);
  }
  esp_task_wdt_add(NULL); 
  displayMessage("Het thoi gian", "Thu lai", 900);
  currentState = STATE_MANAGE_RFID_MENU;
  displayManageRFIDMenu();
}

uint8_t deleteFingerprint(uint8_t id) {
  return finger.deleteModel(id);
}

// =======================================================
// [ĐÃ THÊM] CÁC HÀM CỦA BLYNK
// =======================================================

BLYNK_CONNECTED() { 
  Blynk.syncAll(); 
  Serial.println("Blynk Connected!");
}

// Nút nhấn Mở cửa từ xa (V2)
BLYNK_WRITE(VPIN_REMOTE_UNLOCK) {
  int value = param.asInt();
  if (value == 1 && !isUnlocked) {
    Serial.println("Remote unlock command received!");
    unlockDoor("Remote", false);
    saveUnlockLog("Remote");
    Blynk.virtualWrite(VPIN_REMOTE_UNLOCK, 0); 
  }
}

// [MỚI] Nút nhấn xem Log (V3)
BLYNK_WRITE(VPIN_DUMP_LOG) {
  int value = param.asInt();
  if (value == 1) {
    Serial.println("Đang đọc Log và gửi lên Blynk...");
    Blynk.virtualWrite(VPIN_LOG, "clr"); // Xóa Terminal
    Blynk.virtualWrite(VPIN_LOG, "--- Lich Su Mo Cua ---\n");
    
    uint16_t count = prefs_log.getUShort("log_count", 0);
    uint16_t index = prefs_log.getUShort("log_index", 0);

    if (count == 0) {
      Blynk.virtualWrite(VPIN_LOG, "Log trong.\n");
      return;
    }

    // Tính toán chỉ số bắt đầu đọc (để in từ cũ nhất đến mới nhất)
    uint16_t start_index;
    if (count < MAX_LOG_ENTRIES) {
      start_index = 0; // Log chưa đầy, đọc từ 0
    } else {
      start_index = index; // Log đã đầy, đọc từ vị trí tiếp theo (cũ nhất)
    }

    // Vòng lặp để đọc và in
    for (int i = 0; i < count; i++) {
      uint16_t current_entry_index = (start_index + i) % MAX_LOG_ENTRIES;
      String log_entry = prefs_log.getString(("L" + String(current_entry_index)).c_str(), "");
      
      // Gửi lên Terminal
      Blynk.virtualWrite(VPIN_LOG, String(i + 1) + ": " + log_entry + "\n");
      
      // Thêm delay nhỏ để không làm ngập server Blynk
      delay(10); 
    }
    
    Blynk.virtualWrite(VPIN_LOG, "--------------------\n");
    Serial.println("Đã gửi Log xong.");
  }
}

// [MỚI] Hàm lưu UID vào Log (xoay vòng)
void saveUnlockLog(String uid)  {   //String uid
// Lấy thời gian hiện tại từ NTPClient
  timeClient.update(); // Cập nhật thời gian trước
  if (!timeClient.isTimeSet()) { // Kiểm tra xem đã lấy được giờ chưa
     Serial.println("CẢNH BÁO: Chưa lấy được giờ NTP, không thể thêm timestamp vào log.");
     // Lưu log mà không có timestamp nếu không lấy được giờ
     prefs_log.putString(("L" + String(prefs_log.getUShort("log_index", 0))).c_str(), uid);
     // Phần logic tăng index và count giữ nguyên...
     uint16_t index = prefs_log.getUShort("log_index", 0);
     index = (index + 1) % MAX_LOG_ENTRIES;
     prefs_log.putUShort("log_index", index);
     uint16_t count = prefs_log.getUShort("log_count", 0);
      if (count < MAX_LOG_ENTRIES) {
        count++;
        prefs_log.putUShort("log_count", count);
      }
     return; // Thoát nếu chưa có giờ
  }

  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  // Lấy ngày tháng năm
  unsigned long utcEpochTime = timeClient.getEpochTime(); // Lấy epoch time UTC
  time_t localEpochTime = utcEpochTime + (7 * 3600); // Cộng 7 tiếng (tính bằng giây)
  struct tm *ptm = gmtime(&localEpochTime);
  int currentDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon + 1; // tm_mon bắt đầu từ 0
  int currentYear = ptm->tm_year + 1900; // tm_year là số năm tính từ 1900

  // Định dạng chuỗi ngày tháng năm "DD-MM-YYYY"
  String dateStamp = "";
  if (currentDay < 10) dateStamp += "0";
  dateStamp += String(currentDay) + "-";
  if (currentMonth < 10) dateStamp += "0";
  dateStamp += String(currentMonth) + "-";
  dateStamp += String(currentYear);

  // Định dạng chuỗi thời gian "HH:MM"
  String timeStamp = "";
  if (currentHour < 10) timeStamp += "0";
  timeStamp += String(currentHour) + ":";
  if (currentMinute < 10) timeStamp += "0";
  timeStamp += String(currentMinute);

  // Ghép ngày tháng năm và thời gian vào trước phương thức
  String logEntry = dateStamp + " " + timeStamp + " " + uid;
  Serial.println("Dang luu vao Log: " + logEntry);
  
  // 1. Lấy chỉ số hiện tại
  uint16_t index = prefs_log.getUShort("log_index", 0);
  
  // 2. Ghi đè UID vào vị trí hiện tại
  prefs_log.putString(("L" + String(index)).c_str(), logEntry);
  
  // 3. Tăng và xoay vòng chỉ số
  index = (index + 1) % MAX_LOG_ENTRIES;
  prefs_log.putUShort("log_index", index);
  
  // 4. Cập nhật tổng số lượng (chỉ tăng đến mức tối đa)
  uint16_t count = prefs_log.getUShort("log_count", 0);
  if (count < MAX_LOG_ENTRIES) {
    count++;
    prefs_log.putUShort("log_count", count);
  }
}

// Hàm kiểm tra kết nối (chạy mỗi 5s)
void checkBlynkConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!Blynk.connected()) {
      Serial.println("WiFi connected, connecting to Blynk...");
      Blynk.connect(3000); 
      if(Blynk.connected()) {
        Serial.println("Blynk reconnected!");
      } else {
        Serial.println("Blynk connection failed.");
      }
    }
  } else {
    Serial.println("WiFi disconnected, trying to reconnect...");
    WiFi.reconnect();
  }
}