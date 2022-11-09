#include "esp_adc_cal.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>

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

LiquidCrystal_I2C lcd(0x27, 16, 2);

struct Door{
    int pin = 15; 
    bool open = false;
    long unsigned lastOpen = 0;
    bool changed = false; 
};

Door door; 

// WiFiServer server(80);

void updateDisplay(float tempInC){
    // lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Temp: "+(String)tempInC);
}

void IRAM_ATTR doorInterrupt(){
    long unsigned currentTime = millis();

    if ((currentTime - door.lastOpen) > 300){
        door.open = true;
        door.changed = true; 
        door.lastOpen = currentTime;    
    }

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
    pinMode(door.pin, INPUT_PULLDOWN);
    pinMode(2, OUTPUT);
    attachInterrupt(door.pin, doorInterrupt, FALLING);

    Wire.begin(SDA_LCD, SCL_LCD);
    lcd.init();
    lcd.backlight();
    Serial.begin(115200);
    
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
    float readTemp = getRoomTemp();
    Serial.printf("Temp: %.2f C\n", readTemp);
    
    if (readTemp < 24.0){
        lcd.setCursor(1,1);
        lcd.print("Temp to low..");
        Serial.print("tempto low");
    }

    updateDisplay(readTemp);

    if (door.changed){
        if (door.open){
            Serial.println("Door opennd..");
            door.changed = false; 
        }
    }


    delay(1000);
}
