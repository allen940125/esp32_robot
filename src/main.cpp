#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// // UART2 → GPIO18,19 （保持你现在跑得通的配置）
// #define RX_PIN 18
// #define TX_PIN 19
// HardwareSerial BusSerial(2);

// // STBus 协议
// #define HDR       0x55
// #define CMD_READ  0x1C
// #define CMD_MOVE  0x01
// #define CMD_LOAD  0x1F

// uint8_t chk(const uint8_t* b){
//   uint16_t s=0;
//   for(uint8_t i=2;i<b[3]+2;i++) s+=b[i];
//   return ~s;
// }

// void sendPack(uint8_t id, uint8_t cmd, const uint8_t* p, uint8_t n){
//   uint8_t buf[6+n];
//   buf[0]=buf[1]=HDR;
//   buf[2]=id; buf[3]=n+3; buf[4]=cmd;
//   for(uint8_t i=0;i<n;i++) buf[5+i]=p[i];
//   buf[5+n]=chk(buf);
//   BusSerial.write(buf,6+n);
// }

// void scan(){
//   Serial.println("scan 1~30...");
//   for(uint8_t i=1;i<=30;i++){
//     while(BusSerial.available()) BusSerial.read();
//     sendPack(i,CMD_READ,nullptr,0);
//     delay(30);
//     if(BusSerial.available()>=8){
//       Serial.printf("  ID=%d OK\n",i);
//       while(BusSerial.available()) BusSerial.read();
//     }
//   }
// }

const char* ssid = "ESP32-Access-Point";
const char* password = "aoktiw513";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void handleWebSocketMessage(void arg, uint8_tdata, size_t len) {
  AwsFrameInfo info = (AwsFrameInfo)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0; // Null-terminate the string
    String msg = (char)data;
    Serial.printf("Received message: %s\n", msg.c_str());

    // 解析并执行命令
    if (msg.startsWith("MOVE")) {
      int id, angle, speed;
      if (sscanf(msg.c_str(), "MOVE %d %d %d", &id, &angle, &speed) == 3) {
        if (id >= 1 && id <= 30 && angle >= 0 && angle <= 180 && speed >= -500 && speed <= 500) {
          Serial.printf("Sent MOVE command to ID=%d: angle=%d, speed=%d\n", id, angle, speed);
        } else {
          Serial.println("Invalid parameters");
        }
      } else {
        Serial.println("Invalid command format");
      }
    }
  }
}

void onEvent(AsyncWebSocketserver, AsyncWebSocketClient client, AwsEventType type, voidarg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client connected: %u\n", client->id());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client disconnected: %u\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setup(){
  Serial.begin(115200);
  // BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);

  // WiFi AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.addHandler(&ws);
  ws.onEvent(onEvent);
  server.begin();
  Serial.println("WebSocket server started at ws://<AP_IP>/ws");

  // // 上载扭力
  // uint8_t on=1;
  // sendPack(1,CMD_LOAD,&on,1);
  // delay(50);

  // 扫描 + 初次摆动
  // scan();

  // uint8_t p0[]={0,0,100,0};
  // sendPack(1,CMD_MOVE,p0,4);
  // sendPack(2,CMD_MOVE,p0,4);
  // delay(500);
  // uint8_t p90[]={0x77,0x01,100,0};
  // sendPack(1,CMD_MOVE,p90,4);
  // sendPack(2,CMD_MOVE,p90,4);
}

void loop() {
  ws.cleanupClients();

  // static bool dir = false;
  // uint8_t p0[] = { 0x00,0x00, 100,0x00 };
  // uint8_t p90[] = { 0x77,0x01, 100,0x00 };

  // if (dir) {
  //   sendPack(1, CMD_MOVE, p0, 4);
  //   Serial.println("Servo 1 → Move to 0°");
  //   delay(500);
  //   sendPack(1, CMD_MOVE, p90, 4);
  //   Serial.println("Servo 1 → Move to 90°");
  //   delay(500);
  // } else {
  //   sendPack(2, CMD_MOVE, p0, 4);
  //   Serial.println("Servo 2 → Move to 0°");
  //   delay(500);
  //   sendPack(2, CMD_MOVE, p90, 4);
  //   Serial.println("Servo 2 → Move to 90°");
  //   delay(500);
  // }

  // dir = !dir;
  // delay(800);
}