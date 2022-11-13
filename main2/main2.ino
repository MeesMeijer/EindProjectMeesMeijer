#include "esp_adc_cal.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string>
// #include <WiFi.h>

#define PIN_ANALOG_IN 4                
#define DEFAULT_VREF 1100              
#define NO_OF_SAMPLES 64               

#define BUZZER_PIN 2
#define THERMOLIED_PIN 4 
#define SDA_LCD 19
#define SCL_LCD 18

adc_channel_t channel = ADC_CHANNEL_0; 
adc_unit_t unit = ADC_UNIT_2;
adc_atten_t atten = ADC_ATTEN_DB_11;   
esp_adc_cal_characteristics_t *adc_chars;
esp_adc_cal_value_t val_type;
hw_timer_t *Timer = NULL;
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct Koelkast{
    float highestTemp = 10.0;
    float lowestTemp = 0;

    bool hasUpdate = false; 
};

struct Door{
    int pin = 34; 
    bool open = false;
    long unsigned lastOpen = 0;
    bool changed = false; 
};

struct TimerT{
    int secs = 12;
    int mins = 12;
    int hours = 12;
};


TimerT clock1; 
Door door; 
Koelkast koelkast; 


void IRAM_ATTR doorInterrupt(){
    long unsigned currentTime = millis();

    if ((currentTime - door.lastOpen) > 300){
        door.open = true;
        door.changed = true; 
        door.lastOpen = currentTime;    
    }
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

void updateDisplay(float tempInC){
   
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.printf("%.1f C | | %.i:%.i", tempInC, clock1.hours, clock1.mins);
    lcd.setCursor(0,1);
    lcd.printf("%s", "testing");
}

float getRoomTemp(){
    uint32_t adc_reading = 0;

    for (int i = 0; i < NO_OF_SAMPLES; i++){
        if (unit == ADC_UNIT_1){
            adc_reading += adc1_get_raw((adc1_channel_t)channel);

        } else{
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }

    adc_reading /= NO_OF_SAMPLES;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    double vol = voltage / 1000.0f;
    double Rt = 10 * vol / (3.3 - vol);                             
    double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0); // Temp in kelvin
    double tempC = tempK - 273.15;
    return tempC; 
}


void setup(){
    // Basic setup esp32 
    Serial.begin(115200);

    pinMode(door.pin, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    attachInterrupt(door.pin, doorInterrupt, FALLING);

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


    // ADC callibratie 
    if (unit == ADC_UNIT_1){  
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten((adc1_channel_t)channel, atten);
    }
    else{
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1,sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

void loop(){
    // true every second. 
    if (koelkast.hasUpdate){
        float readTemp = getRoomTemp();
        updateDisplay(readTemp);
        
        Serial.printf("Temp: %.2f C\n", readTemp);
        
        if (readTemp > koelkast.highestTemp){
            Serial.print("temp to HIGH\n");
        }
    
        koelkast.hasUpdate = false;
    }
    if (door.changed){
        if (door.open){
            Serial.println("Door opennd..");
            door.changed = false; 
        }
    }
    // delay(1000);
    
}
