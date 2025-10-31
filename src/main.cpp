#include <Arduino.h>
#include <vector>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#pragma region 網路

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

#pragma endregion

// Create a struct_message called myData
float myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}

// put function declarations here:
// UART2 → GPIO18,19 （保持你现在跑得通的配置）
#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);

// STBus 协议
#define HDR       0x55
#define CMD_READ  0x1C
#define CMD_MOVE  0x01
#define CMD_LOAD  0x1F
#define CMD_MODE  0x1D
#define CMD_WRITE_ID  0x0D

#pragma region 輸入端

uint8_t chk(const uint8_t* b){
  uint16_t s=0;
  for(uint8_t i=2;i<b[3]+2;i++) s+=b[i];
  return ~s;
}

void sendPack(uint8_t id, uint8_t cmd, const uint8_t* p, uint8_t n){
  uint8_t buf[6+n];
  buf[0]=buf[1]=HDR;
  buf[2]=id; buf[3]=n+3; buf[4]=cmd;
  for(uint8_t i=0;i<n;i++) buf[5+i]=p[i];
  buf[5+n]=chk(buf);
  BusSerial.write(buf,6+n);
}

void scan(){
  Serial.println("scan 1~30...");
  for(uint8_t i=1;i<=30;i++){
    while(BusSerial.available()) BusSerial.read();
    sendPack(i,CMD_READ,nullptr,0);
    delay(30);
    if(BusSerial.available()>=8){
      Serial.printf("  ID=%d OK\n",i);
      while(BusSerial.available()) BusSerial.read();
    }
  }
}

#pragma endregion

#pragma region 10進位轉換16進位

// 把一個 int 轉換成小端序 bytes (16-bit)
std::vector<uint8_t> intToBytes(uint16_t value) {
  std::vector<uint8_t> bytes(2);
  bytes[0] = value & 0xFF;        // 低位
  bytes[1] = (value >> 8) & 0xFF; // 高位
  return bytes;
}

// 可接受任意多個 int，拼接成一個 byte 陣列
template<typename... Args>
std::vector<uint8_t> makeBytes(Args... values) {
  std::vector<uint8_t> result;
  ((
    [&] {
      auto b = intToBytes((uint16_t)values);
      result.insert(result.end(), b.begin(), b.end());
    }()
  ), ...);
  return result;
}

// 輔助函式：列印陣列
void printHexArray(const std::vector<uint8_t>& arr) {
  Serial.print("{ ");
  for (size_t i = 0; i < arr.size(); ++i) {
    Serial.print("0x");
    if (arr[i] < 16) Serial.print("0");
    Serial.print(arr[i], HEX);
    if (i < arr.size() - 1) Serial.print(", ");
  }
  Serial.println(" }");
}

#pragma endregion

#pragma region 角度

// 把角度(度)與時間(ms)轉成小端序 4 bytes (angleLow, angleHigh, timeLow, timeHigh)
std::vector<uint8_t> buildAngleBytes(double angleDegrees, uint16_t timeMs) {
    // 1 unit = 0.24°，計算角度單位
    int angleUnits = static_cast<int>(std::round(angleDegrees / 0.24));
    if (angleUnits < 0) angleUnits = 0;
    if (angleUnits > 1000) angleUnits = 1000; // clamp 最大值

    if (timeMs > 30000) timeMs = 30000; // clamp 最大時間

    std::vector<uint8_t> result;
    // 角度兩個 byte
    auto angleBytes = intToBytes(static_cast<uint16_t>(angleUnits));
    result.insert(result.end(), angleBytes.begin(), angleBytes.end());
    // 時間兩個 byte
    auto timeBytes = intToBytes(timeMs);
    result.insert(result.end(), timeBytes.begin(), timeBytes.end());

    return result; // 總長 4 bytes
}

#pragma endregion

std::vector<uint8_t> motor1 = makeBytes(1, -900);
std::vector<uint8_t> motor2 = makeBytes(1, 1000);

#pragma region 執行端

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);

  //wifi

  WiFi.mode(WIFI_STA);  //設定 WiFi的模式為 STA，共有 WIFI_AP、WIFI_STA、WIFI_AP_STA、WIFI_OFF。
  WiFi.begin();

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();

  // 上载扭力
  uint8_t on=1;
  sendPack(1,CMD_LOAD,&on,1);
  delay(50);

  // 扫描 + 初次摆动
  scan();

  //測試旋轉

  auto p90 = buildAngleBytes(90.0, 100);   // 90°，100ms
  sendPack(1, CMD_MOVE, p90.data(), p90.size());
  sendPack(2, CMD_MOVE, p90.data(), p90.size());

  delay(500);

  auto p180 = buildAngleBytes(180.0, 500); // 180°，500ms
  sendPack(1, CMD_MOVE, p180.data(), p180.size());
  sendPack(2, CMD_MOVE, p180.data(), p180.size());

  delay(500);

  //測試移動

  //sendPack(1, CMD_MODE, motor1, 4);
  //sendPack(2, CMD_MODE, motor2, 4);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  myData = 0.0;

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() 
{
  if(myData > 40)
  {
    motor1 = makeBytes(1, -900);
    motor2 = makeBytes(1, 1000);
  }
  else if(myData > 30)
  {
    motor1 = makeBytes(1, -900 * 0.8f);
    motor2 = makeBytes(1, 1000 * 0.8f);
  }
  else if(myData > 20)
  {
    motor1 = makeBytes(1, -900 * 0.5f);
    motor2 = makeBytes(1, 1000 * 0.5f);
  }
  else
  {
    motor1 = makeBytes(1, 0);
    motor2 = makeBytes(1, 0);
  }

  sendPack(1, CMD_MODE, motor1.data(), motor1.size());
  sendPack(2, CMD_MODE, motor2.data(), motor2.size());
  Serial.print("Distance: ");
  Serial.println(myData);

  delay(500);
}

#pragma endregion

