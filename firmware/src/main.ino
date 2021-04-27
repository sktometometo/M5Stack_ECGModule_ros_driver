/*
    Description: Heart rate statistics, and display ECG on the screen.

    Original source code is https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Module/ECG/ECG.ino

    Modified by sktometometo ( sktometometo@gmail.com )
*/
#include <M5Stack.h>

#include <ros/node_handle.h>
#include <BluetoothHardware.h>
#include <std_msgs/UInt16.h>

#include "ECG.h"

ECG ecg;

ros::NodeHandle_<BluetoothHardware, 25, 25, 4096, 4096> nh;
std_msgs::UInt16 msg_data;
std_msgs::UInt16 msg_heart_rate;
ros::Publisher publisher_data("~data", &msg_data);
ros::Publisher publisher_heart_rate("~heart_rate", &msg_heart_rate);

void setup() {
    M5.begin(true,false,true,false);

    Serial.begin(57600);
    nh.initNode("M5Stack ECG ROS");
    nh.advertise( publisher_data );
    nh.advertise( publisher_heart_rate );
    M5.Lcd.printf("Connecting rosserial\n");
    while ( not nh.connected() ) {
        nh.spinOnce();
    }
    M5.Lcd.printf("rosserial connected\n");

    M5.Lcd.printf("Initialized.\n");
}


void loop() {
    ecg.readData();

    uint16_t data = ecg.getData(0);
    msg_data.data = data;
    publisher_data.publish(&msg_data);

    uint16_t heart_rate = ecg.getHeartRate();
    msg_heart_rate.data = heart_rate;
    publisher_heart_rate.publish(&msg_heart_rate);

    nh.spinOnce();

    delay(10);
    M5.update();
}
