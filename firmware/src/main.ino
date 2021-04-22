/*
    Description: Heart rate statistics, and display ECG on the screen.

    Original source code is https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Module/ECG/ECG.ino

    Modified by sktometometo ( sktometometo@gmail.com )
*/
#include <M5Stack.h>
#include "ECG.h"

ECG ecg;
//TFT_eSprite TFTChart = TFT_eSprite(&M5.Lcd);
//TFT_eSprite TFTNum = TFT_eSprite(&M5.Lcd);

void setup() {
    M5.begin(true,false,true,false);
    M5.Power.begin();

    Serial.begin(115200);

    //TFTChart.createSprite(320,100);
    //TFTChart.fillRect(0,0,320,100,BLUE);
    //TFTChart.pushSprite(0,100);

    //TFTNum.createSprite(150,100);
    //TFTNum.fillRect(0,0,150,100,BLACK);
    //TFTNum.fillRect(0,0,20,100,RED);
    //TFTNum.pushSprite(0,0);
}


void loop() {
    ecg.readData();
    //ecg.display(TFTChart,TFTNum);
    uint16_t data = ecg.getData(0);
    Serial.println(data);;
    delay(5);
    M5.update();
}
