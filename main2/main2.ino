
#include "esp_adc_cal.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WebSocketsClient.h>

#include <string>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>

#define PIN_ANALOG_IN 34              
#define DEFAULT_VREF 1100              
#define NO_OF_SAMPLES 64               

#define BUZZER_PIN 2
#define THERMOLIED_PIN 34 
#define SDA_LCD 19
#define SCL_LCD 18
#define LED_PIN 32


hw_timer_t *Timer = NULL;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Preferences localStorage;
WebSocketsClient webSocket;


enum State{
    STARTING = 0,
    ERROR = 1,
    
    CONNECTED = 2,
    RE_CONNECTING = 3,
    DISCONNECTED = 4,

    TO_HIGH = 5,
    TO_LOW = 6,

    GOT_IP = 7,
    OPEN_DOOR = 8,

    NORMAL = 9,
};

struct Object{
    float highestTemp = 24.0;
    float lowestTemp = 22.0;

    bool hasUpdate = false;

    int state = State::NORMAL;
    int wsState = State::NORMAL;
    int doorState = State::NORMAL; 
};

struct Door{
    int pin = 35; 
    bool before; 
    bool open = false;
    long unsigned lastOpen;
    bool changed = false; 
};

struct TimerT{
    int secs = 12;
    int mins = 12;
    int hours = 12;
};

TimerT clock1; 
Door door; 
Object koelkast; 

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16)
{
    const uint8_t *src = (const uint8_t *)mem;
    Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
    for (uint32_t i = 0; i < len; i++)
    {
        if (i % cols == 0)
        {
            Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
        }
        Serial.printf("%02X ", *src);
        src++;
    }
    Serial.printf("\n");
}


void handleSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
    String strLoad = "";

    switch (type){
        case WStype_DISCONNECTED:
            Serial.printf("[WSc] Disconnected!\n");
            koelkast.wsState = State::DISCONNECTED;
            break;
        case WStype_CONNECTED:
            Serial.printf("[WSc] Connected to url: %s\n", payload);
            webSocket.sendTXT("Connected");
            koelkast.wsState = State::CONNECTED;
            break;

        case WStype_TEXT:
            Serial.printf("[WSc] get text: %s\n", payload);

            strLoad = (char* )payload;
            // SET_TEMP: 14.0 | 19.0
            if (strLoad.startsWith("AUTH")){
                Serial.println("Authentication send");
                webSocket.sendTXT("AUTH:SECRET_TOKEN");

            } else if (strLoad.startsWith("SET_TEMP_MIN")){
                strLoad.replace("SET_TEMP_MIN:", "");
                koelkast.lowestTemp = strLoad.toFloat();
                Serial.println("Min now "+ (String) koelkast.lowestTemp + " been"+ strLoad);

            }else if (strLoad.startsWith("SET_TEMP_MAX")){
                strLoad.replace("SET_TEMP_MAX:", "");
                koelkast.highestTemp = strLoad.toFloat();
                Serial.println("Max now "+ (String) koelkast.highestTemp + " been"+ strLoad);

            }else if (strLoad.startsWith("SET_HOUR")){
                strLoad.replace("SET_HOUR:", "");
                clock1.hours = strLoad.toInt();
                Serial.println("Hours now "+ (String)clock1.hours + " been"+ strLoad);

            }else if (strLoad.startsWith("SET_MINS")){
                strLoad.replace("SET_MINS:", "");
                clock1.mins = strLoad.toInt();
                Serial.println("Mins now "+ (String)clock1.mins + " been"+ strLoad);

            }else if (strLoad.startsWith("SET_SEC")){
                strLoad.replace("SET_SEC:", "");
                clock1.secs = strLoad.toInt();
                Serial.println("Secs now "+ (String)clock1.secs + " been"+ strLoad);
            }
            break;
        case WStype_BIN:
            Serial.printf("[WSc] get binary length: %u\n", length);
            hexdump(payload, length);
            break;

        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}

// void IRAM_ATTR doorInterrupt(){
//     long unsigned currentTime = millis();

//     // if ((currentTime - door.lastOpen) > 1000){
//         door.open = true;
//         door.changed = true; 
//         door.lastOpen = currentTime;    
//     // }
// }

void IRAM_ATTR doorDicht(){
    door.open = false;
    door.changed = true; 
    door.lastOpen = millis();
}

void IRAM_ATTR doorOpen(){
    door.open = true; 
    door.changed = true; 
    door.lastOpen = millis();
}

void IRAM_ATTR timerInterrupt(){
    clock1.secs += 1;

    if (clock1.secs >= 60){
        clock1.secs = 0;
        clock1.mins += 1;

        if (clock1.mins >= 60) {
            clock1.mins = 0; 
            clock1.hours += 1;

            if (clock1.hours >= 24){
                clock1.hours = 0;
            }
        }
    }

    koelkast.hasUpdate = true; 
}

void updateDisplay(float tempInC = 0.0){
   
    lcd.clear();
    lcd.setCursor(0,0);

    String mins = (String)clock1.mins;
    String Hours = (String)clock1.hours;

    if (clock1.mins < 10.0){
        mins = "0" + mins;
    }
    if (clock1.hours < 10.0){
        Hours = "0"+Hours;
    }

    lcd.printf("%.1f C   %s:%s", tempInC, Hours, mins);
    lcd.setCursor(0,1);

    switch (koelkast.wsState){
        case State::STARTING:
            lcd.printf("%s", "Starting..");
            break;

        case State::CONNECTED:
            lcd.printf("%s", "Ws Connected");
            koelkast.wsState = State::NORMAL;
            break;

        case State::RE_CONNECTING:
            lcd.printf("%s", "Re-connecting");
            break;    

        case State::DISCONNECTED:
            lcd.printf("%s", "Disconnected");
            break;

        case State::TO_HIGH:
            lcd.printf("%s", "TO HIGH");
            break;

        case State::TO_LOW:
            lcd.printf("%s", "TO LOW");
            break;

        case State::ERROR:
            lcd.printf("%s", "Error..");
            break;

        case State::GOT_IP:
            lcd.printf("IP: %s", WiFi.localIP());
            break;
        
        case State::OPEN_DOOR:
            lcd.printf("%s", "Door Open");
            break;
        
        case State::NORMAL:
            lcd.printf("%s", "Normal");
            break;
    };
}

float getRoomTemp(){
    uint32_t voltage = analogReadMilliVolts(34);
    double vol = voltage / 1000.0f;
    double Rt = 10 * vol / (3.3 - vol);                             
    double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0); // Temp in kelvin
    double tempC = tempK - 273.15;
    return tempC; 
}

void setup(){
    WiFi.mode(WIFI_STA);

    // Basic setup esp32 
    Serial.begin(115200);

    pinMode(door.pin, INPUT_PULLDOWN);
    pinMode(BUZZER_PIN, OUTPUT);

    attachInterrupt(door.pin, doorDicht, FALLING);
    attachInterrupt(door.pin, doorOpen,  RISING);

    // setup lcd op pin 18, 19
    Wire.begin(SDA_LCD, SCL_LCD);
    lcd.init();
    lcd.backlight();

    // Timer setup
    Timer = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer, timerInterrupt,  true);
    // 1000000 uS == 1 sec
    timerAlarmWrite(Timer, 1000000, true);
    timerAlarmEnable(Timer);

    WiFiManager wm;
    bool res;

    res = wm.autoConnect("AutoConnectAP","password");
    if (!res){
        Serial.println("Coudnt connected..");
    }else{
        // Serial.println(WiFi.isConnected());
        Serial.println("Conencted to Wifi");
    }

    webSocket.enableHeartbeat(1000, 6000, 0);
    webSocket.beginSSL("project1.meesinc.nl", 443);
    webSocket.onEvent(handleSocketEvent);
    webSocket.setExtraHeaders("AUTHENTICATION: SECRET_TOKEN_2022");
    webSocket.setExtraHeaders("UUID: 1");
    webSocket.setReconnectInterval(1000);
}

void loop(){
    webSocket.loop();
    
    // true every second. 
    if (koelkast.hasUpdate){
        float readTemp = getRoomTemp();
        koelkast.wsState = State::NORMAL;

        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);

        if (door.changed){
            if (door.open) koelkast.wsState = State::OPEN_DOOR;
            door.changed = false; 
            
        } else {
            koelkast.wsState = State::NORMAL;
        }
        
        if (readTemp > koelkast.highestTemp){
            // koelkast.wsState = State::TO_HIGH;

            if (door.open && ( (millis() - door.lastOpen) < (2 * 60 * 1000))){
                Serial.println("Door open for to long..");
            }

        }else if (readTemp < koelkast.lowestTemp){
            // koelkast.wsState = State::TO_LOW;

            if (( (millis() - door.lastOpen) > (2 * 60 * 1000))){
                // Serial.println("Koelkast is kapot.. check de deur!");
            }
        }

        if (webSocket.isConnected()){
            webSocket.sendTXT("READINGS:"+(String)readTemp+ "|"+ (String)door.open);
        }
        
        updateDisplay(readTemp);
        koelkast.hasUpdate = false;
        // door.open = false; 
        door.open = digitalRead(door.pin);
        Serial.println(door.open);
    }

    if (getRoomTemp() > koelkast.highestTemp && (millis() - door.lastOpen) > (10*1000)){ 
       digitalWrite(BUZZER_PIN, HIGH);
    }
    
    // delay(1000);
    
}
