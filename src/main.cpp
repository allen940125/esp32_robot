#include <Arduino.h>
#include <vector>

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

#pragma region 執行端

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);

  // 上载扭力
  uint8_t on=1;
  sendPack(1,CMD_LOAD,&on,1);
  delay(50);

  // 扫描 + 初次摆动
  scan();

  //測試旋轉

  // uint8_t op=2;
  // sendPack(1, CMD_WRITE_ID, &op, 1);

  // uint8_t p0[]={0,0,100,0};
  // sendPack(1,CMD_MOVE,p0,4);
  // sendPack(2,CMD_MOVE,p0,4);
  // delay(500);
  // uint8_t p90[]={0x77,0x01,100,0};
  // sendPack(1,CMD_MOVE,p90,4);
  // sendPack(2,CMD_MOVE,p90,4);

  auto p90 = buildAngleBytes(90.0, 100);   // 90°，100ms
  sendPack(1, CMD_MOVE, p90.data(), p90.size());
  sendPack(2, CMD_MOVE, p90.data(), p90.size());

  delay(500);

  auto p180 = buildAngleBytes(180.0, 500); // 180°，500ms
  sendPack(1, CMD_MOVE, p180.data(), p180.size());
  sendPack(2, CMD_MOVE, p180.data(), p180.size());

  delay(500);

  //測試移動

  //std::vector<uint8_t> p180 = makeBytes(180);
  
  //uint8_t motor1[] = {0x01, 0, 0xF4, 0x01};
  //uint8_t motor2[] = {0x01, 0, 0x0C, 0xFE};

  //sendPack(1, CMD_MODE, motor1, 4);
  //sendPack(2, CMD_MODE, motor2, 4);

  std::vector<uint8_t> motor1 = makeBytes(1, 1000);
  std::vector<uint8_t> motor2 = makeBytes(1, -1000);

  sendPack(1, CMD_MODE, motor1.data(), motor1.size());
  sendPack(2, CMD_MODE, motor2.data(), motor2.size());
}

void loop() {
  // put your main code here, to run repeatedly:
  // uint8_t steps[4][4] = {{0x00, 0x00, 0xF4, 0x01}, {0x77, 0x01, 0xF4, 0x01}, {0xEE, 0x02, 0xF4, 0x01}, {0xE8, 0x03, 0xF4, 0x01}};

  // for(int i = 0; i < 4; i++)
  // {
  //   sendPack(1, CMD_MOVE, steps[i], 4);
  //   Serial.printf("Move to %d°\n", i*90);
  //   delay(1000);
  // }
}

#pragma endregion

