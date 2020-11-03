//-----------------Libraries-----------------
//Libraries for WiFi
#include "WiFi.h"

//Libraries for MQTT Client
#include <PubSubClient.h>

//Libraries for Message Handling
#include <ArduinoJson.h>
#include <string.h>

//Libraries for I2C LCD Display
#include <LiquidCrystal_I2C.h>

//Libraries for SD Card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

//Libraries for Temperature Sensor
#include <dht11.h>

//WiFi ADC2 Fix
#include <soc/sens_reg.h>

#include <bits/stdc++.h>
//----------------- Function Prototypes------
void mqtt_message(char *topic, byte *payload, unsigned int length);

//-----------------Variables-----------------
//WiFi
WiFiClient wifiClient;
String pass = "";
String ssid = "";

//MQTT Variables
String broker_port = "";
String broker_username = "";
String broker_address = "";
String broker_password = "";
String broker_subscription = "";
String broker_calibrate = "hub/calib/";
PubSubClient mqtt_client(wifiClient);

//Allocated Devices Pins
#define DEV_1 17 //1
#define DEV_2 16 //2
#define DEV_3 4  //5
#define DEV_6 0  //6
#define DEV_5 2  //5
#define DEV_7 15 //7
#define DEV_4 14 //4
#define DEV_8 12 //8

//Allocated Sensor Pins
#define CS_1 32
#define VS_1 33
#define TS_1 34

//SD CARD Variables
#define SD_MOSU 23
#define SD_CS 5
#define SD_MISO 19
#define SD_CLK 18

//LCD Display Variables
LiquidCrystal_I2C lcd(0x20, 8, 1);

//Tempature Sensor Variable
dht11 DHT11;
float temp_sense = 0.0;

//Main Loop Counter
uint32_t counter_lcd = 0;

uint8_t disp_counter = 1;
String a = WiFi.localIP().toString();
String ip_hold = "";
uint8_t period_counter = 0;

//Current Sensor Offset
float CS_1_OS = 0.0;

//Sensor Value Accumulators

float avg_curr = 0.0;
float avg_volt = 0.0;
float avg_temp = 0.0;
uint64_t prevPub = 0;
uint64_t prevSample = 0;

//-----------------Setup-----------------
void setup()
{
    //LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("STARTUP");

    //Serial
    Serial.begin(9600);
    Serial.println("Initilizing");
    LCD_Print("Init...");

    analogReadResolution(10);
    analogSetAttenuation((adc_attenuation_t)2);

    //SD Card
    SD.begin(SD_CS);
    while (!SD.begin(SD_CS) || (SD.cardType() == CARD_NONE))
    {

        if (SD.cardType() == CARD_NONE)
        {
            Serial.println("SD Card Error - Card Not Found");
            LCD_Print("SD_Error");
            delay(1000);
        }
        else
        {
            if (!SD.begin(SD_CS))
            {
                Serial.println("SD Card Error - Initilization Failed");
                LCD_Print("SD_Error");
                delay(1000);
            }
        }
    }

    LCD_Print("SD_Init");
    Serial.println("Initializing SD Card.");
    SD.mkdir("/devices");
    SD.mkdir("/devices/dev");
    SD.mkdir("/credentials");
    SD.mkdir("/credentials/wifi");
    SD.mkdir("/credentials/mqtt");

    //Load Credentials
    GetCredentials(SD);
    String hold_str = "";

    for (uint8_t i = 1; i < 9; i++)
    {
        //--------------------------Link Device to Device Pin---------------------------------------------
        //---------File Name:       Device Number
        //---------File Contents:   Device Pin

        hold_str = "/devices/dev/" + String(i) + ".txt";
        switch (i)
        {
        case 1:
            writeFile(SD, hold_str.c_str(), String(DEV_1).c_str());
            break;
        case 2:
            writeFile(SD, hold_str.c_str(), String(DEV_2).c_str());
            break;
        case 3:
            writeFile(SD, hold_str.c_str(), String(DEV_3).c_str());
            break;
        case 4:
            writeFile(SD, hold_str.c_str(), String(DEV_4).c_str());
            break;
        case 5:
            writeFile(SD, hold_str.c_str(), String(DEV_5).c_str());
            break;
        case 6:
            writeFile(SD, hold_str.c_str(), String(DEV_6).c_str());
            break;
        case 7:
            writeFile(SD, hold_str.c_str(), String(DEV_7).c_str());
            break;
        case 8:
            writeFile(SD, hold_str.c_str(), String(DEV_8).c_str());
            break;
        default:
            break;
        }
    }

    //Wifi
    LCD_Print("WiFi_Ini");
    Serial.println("Wifi Initializing");
    WiFi.begin(ssid.c_str(), pass.c_str());
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.print("\n");

    //MQTT
    LCD_Print("MQTT_Ini");
    Serial.println("MQTT Initializing");
    mqtt_client.setServer(broker_address.c_str(), broker_port.toInt());
    mqtt_client.setCallback(mqtt_message);
    mqtt_reconnect();

    //Relay
    pinMode(DEV_1, OUTPUT);
    pinMode(DEV_2, OUTPUT);
    pinMode(DEV_3, OUTPUT);
    pinMode(DEV_4, OUTPUT);
    pinMode(DEV_5, OUTPUT);
    pinMode(DEV_6, OUTPUT);
    pinMode(DEV_7, OUTPUT);
    pinMode(DEV_8, OUTPUT);

    //Relay Off
    Serial.println("Relays Off");
    LCD_Print("RelayOff");
    RelaySetAll(1);

    //Calibrate Current Sensors

    CalibrateCurrentSensor();

    //Final
    LCD_Print("Ready!__");
}

void loop()
{

    mqtt_client.loop();

    //---------------------------------------------------------------------------
    //-------------------------Cycling Stats-------------------------------------
    //---------------------------------------------------------------------------
    if ((millis() - counter_lcd) > 10000)
    {
        counter_lcd = millis();
        switch (disp_counter % 5)
        {
        case 1:
            LCD_Print(String(avg_temp) + (char)223 + "C");
            Serial.println("Internal Temperature = " + String(avg_temp) + (char)223 + "C");
            break;
        case 2:
            a = WiFi.localIP().toString();
            ip_hold = "";
            period_counter = 0;
            for (uint8_t i = 0; i < a.length(); i++)
            {
                if (String(a[i]) == String("."))
                {
                    period_counter++;
                }
                if (period_counter >= 3)
                {
                    ip_hold += String(a[i]);
                }
            }
            LCD_Print("IP:*" + ip_hold);
            Serial.println("Device IP:\t\t " + ip_hold);
            break;
        case 3:
            LCD_Print(deviceState());
            Serial.println("Device States:\t\t " + deviceState());
            break;
        case 4:
            LCD_Print(broker_subscription);
            Serial.println("Subscribed to:\t\t " + broker_subscription);
            disp_counter = 0;
            break;
        default:
            break;
        }
        disp_counter++;
    }

    //---------------------------------------------------------------------------
    //-------------------------Data Sampling-------------------------------------
    //---------------------------------------------------------------------------
    if ((millis() - prevSample) > 10)
    {
        prevSample = millis();
        DataSampling();
    }
    if ((millis() - prevPub) > 1000)
    {
        prevPub = millis();
        dataPush();
    }
}

/*
    Converts the states of all devices relay pins, and converts them into a String

    return - String of enabled relays.

    */
String deviceState()
{
    String states = "";
    uint8_t stat = 0;
    if (digitalRead(DEV_1) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    if (digitalRead(DEV_2) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    if (digitalRead(DEV_3) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    if (digitalRead(DEV_4) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    if (digitalRead(DEV_5) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    if (digitalRead(DEV_6) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    if (digitalRead(DEV_7) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    if (digitalRead(DEV_8) == stat)
    {
        states += "X";
    }
    else
    {
        states += "_";
    }
    return states;
}

/*
    Control the Power State of the Relay

    device - Hub device number to control.

    return - None.

    Note:
    The device number used is the non-unique number.
    Not the device's ID.

    */
void RelayOn(uint8_t device)
{
    uint8_t pin = getDevicePin(device);

    digitalWrite(pin, 0);
    Serial.print("DEV");
    Serial.print(device);
    Serial.println("-ON_");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DEV");
    lcd.print(device);
    lcd.print("-ON_");
}
void RelayOff(uint8_t device)
{
    uint8_t pin = getDevicePin(device);

    digitalWrite(pin, 1);
    Serial.print("DEV");
    Serial.print(device);
    Serial.println("-OFF");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DEV");
    lcd.print(device);
    lcd.print("-OFF");
}

/*
    Control Print on LCD Display

    message - Message to Print

    return - None.

*/
void LCD_Print(String message)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message);
}

/*
    Get Credentials of WiFi and MQTT

    fs - SD Card

    return - None.

    Note:
    Relevant credentials must be placed in the following locations
    MQTT Username -              /credentials/mqtt/user.txt
    MQTT Password -              /credentials/mqtt/pass.txt
    MQTT Broker Address -        /credentials/mqtt/broker.txt
    MQTT Subscribed Channel -    /credentials/mqtt/subscribe.txt
    WiFi SSID -                  /credentials/wifi/ssid.txt
    WiFi Password -              /credentials/wifi/pass.txt

*/
void GetCredentials(fs::FS &fs)
{
    //Wifi
    ssid = ReadData(String("/credentials/wifi/ssid.txt"));
    pass = ReadData(String("/credentials/wifi/pass.txt"));
    //MQTT
    broker_username = ReadData(String("/credentials/mqtt/user.txt"));
    broker_password = ReadData(String("/credentials/mqtt/pass.txt"));
    broker_address = ReadData(String("/credentials/mqtt/broker.txt"));
    broker_subscription = ReadData(String("/credentials/mqtt/subscribe.txt"));
    broker_port = ReadData(String("/credentials/mqtt/port.txt"));
    broker_calibrate = "hub/calib/" + ReadData("/credentials/mqtt/id.txt");
    Serial.println("Credentials Loaded");
}

/*
    Setup the WiFi connection

    return - None.

*/
void WiFi_Connect()
{
    WiFi.begin(ssid.c_str(), pass.c_str());
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(10);
    }
}

/*
    Transmits MQTT Message
    subscription - Topic of published message

    JSON_Buffer - JSON Buffer to transmit

    return - None.

*/
void mqtt_publish(StaticJsonDocument<300> JSON_Buffer, String subscription)
{
    char Payload_Char[300];
    size_t n = serializeJson(JSON_Buffer, Payload_Char);
    if (mqtt_client.publish(subscription.c_str(), Payload_Char, n) == true)
    {
    }
    else
    {
        mqtt_reconnect();
        Serial.println("Failed to Send Message.");
    }
}

/*
    MQTT Message Handler Function

    topic - Topic of recieved message

    payload - message recieved. This is in a JSON format, and will need to be manually managed.

    length - length of payload.

    return - None.

*/
void mqtt_message(char *topic, byte *payload, unsigned int length)
{
    String hub_id = ReadData("/credentials/mqtt/id.txt");
    String hub_calibration = "hub/calib/" + hub_id;
    String payload_str;
    StaticJsonDocument<500> jsonBuffer;
    DeserializationError error = deserializeJson(jsonBuffer, payload);

    //Array [Device Number][Device ID]
    uint8_t hold_val = 0;
    StaticJsonDocument<300> Payload_JSON;
    char Payload_Char[300];
    String broker_calibration = "";

    if (error)
    {
        Serial.println(error.c_str());
        return;
    }

    //=================================================================Process Payload
    const char *element_1 = jsonBuffer["0"]; //IP ADDRESS
    const char *element_2 = jsonBuffer["1"]; //HUB ID       |   Defined by Server
    const char *element_3 = jsonBuffer["2"]; //Hub Pin      |   Device Pin
    const char *element_4 = jsonBuffer["3"]; //State        |   State

    //=================================================================Change Hub ID

    if ((String(topic) == "hub/init") && (WiFi.localIP().toString() == String(element_1)))
    {
        payload_str = (String) "hub/" + String(element_2);
        mqtt_client.subscribe(payload_str.c_str());
        mqtt_client.unsubscribe(broker_subscription.c_str());

        String hold_calibrate = "hub/calib/" + String(element_2);
        mqtt_client.subscribe(hold_calibrate.c_str());
        mqtt_client.unsubscribe(broker_calibrate.c_str());

        //Serial Print
        Serial.print("Subscibed to: ");
        Serial.println(payload_str);
        Serial.print("Unsubscribed from: ");
        Serial.println(broker_subscription);
        broker_calibrate = hold_calibrate;
        broker_subscription = payload_str;
        writeFile(SD, "/credentials/mqtt/subscribe.txt", payload_str.c_str());
        writeFile(SD, "/credentials/mqtt/id.txt", String(element_2).c_str());
    }
    //=================================================================Manage Device
    else if ((String(topic) == broker_subscription) && (WiFi.localIP().toString() == String(element_1)))
    {
        //Check if wantingto calibrate
        //Calibration requres State = X | Device = 0
        if (String(element_3[0]) == String(0))
        {
            Serial.println("");
        }
        //Control Device State
        //Device Control requres State = 1 or 0 | Device = [1 - 8]
        else if ((String(element_4[0]) == String(1)) && ((String(element_3[0]) > String(0)) && (String(element_3[0]) < String(9))))
        {
            RelayOn(String(element_3[0]).toInt());
        }
        else if ((String(element_4[0]) == String(0)) && ((String(element_3[0]) > String(0)) && (String(element_3[0]) < String(9))))
        {
            RelayOff(String(element_3[0]).toInt());
        }
    }
}

/*
    MQTT Connect Client

    return - None.

*/
void mqtt_reconnect()
{
    while (!mqtt_client.connected())
    {
        String ClientID = String(random(0xffff), HEX);

        if (mqtt_client.connect(ClientID.c_str(), broker_username.c_str(), broker_password.c_str()))
        {
            mqtt_client.subscribe("hub/init");
            mqtt_client.subscribe(broker_subscription.c_str());
            mqtt_client.subscribe(broker_calibrate.c_str());
        }
        else
        {
            delay(5000);
            Serial.print("Attempting MQTT connection...\n");
        }
    }
}

/*
    Writes information to SD Card
    fs      -   SD Card handle
    path    -   path directory + extension
    message -   Data to be writent to file
    
    return - None.

*/
void writeFile(fs::FS &fs, const char *path, const char *message)
{
    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to Open File");
        return;
    }
    if (file.print(message))
    {
        Serial.print("Successfully Created - ");
        Serial.println(path);
    }
    else
    {
        Serial.print("Creation Failed - ");
        Serial.println(path);
    }
    file.close();
}

/*
    Reads Information from a defined file
    path    -   path directory + extension
        
    return - Data inside file.

*/
String ReadData(String path)
{
    File file = SD.open(path.c_str());
    String hold_str = "";
    if (!file)
    {
        Serial.println("Failed to open file for reading - " + path);
    }
    while (file.available())
    {
        hold_str += (char)file.read();
    }
    file.close();
    return String(hold_str);
}

/*
    Dynamically updates rolling average
    mean    -   Current average
    newVal  -   Newly measured value
        
    return - New updated average.

*/
float rollAverage(float mean, uint16_t newVal)
{
    uint8_t N = 100;
    mean -= (float)mean / N;
    mean += (float)newVal / N;
    return mean;
}

/*
    Samples data (temperature / voltage / current)
        
    return - Updated rolling averages for all measurements.

*/
void DataSampling()
{
    prevSample = millis();

    uint16_t Curr = analogRead(CS_1);
    uint16_t Volt = analogRead(VS_1);
    uint16_t Temp = (float)DHT11.temperature;

    avg_curr = rollAverage(avg_curr, Curr);
    avg_volt = rollAverage(avg_volt, Volt);
    avg_temp = rollAverage(avg_temp, Temp);
}

/*
    Pushes data to MQTT
        
    return - Published MQTT Data
*/
void dataPush()
{
    StaticJsonDocument<300> Payload_JSON;
    char Payload_Char[300];
    float Vref = 3.3;
    float buffSize = 1023.0;

    String subscription = "sensors/" + ReadData("/credentials/mqtt/id.txt");
    //Publishing Payload
    Payload_JSON["ID"] = (ReadData("/credentials/mqtt/id.txt")).toInt();
    Payload_JSON["CURR"] = (float)(CS_1_OS - avg_curr) * (Vref / buffSize) / 0.066;
    Payload_JSON["VOLT"] = (float)avg_volt * (Vref / buffSize);
    Payload_JSON["TEMP"] = (float)avg_temp;
    mqtt_publish(Payload_JSON, subscription);
    prevPub = millis();
}

/*
    Calibrates Current Sensor, and gets an offset value
*/
void CalibrateCurrentSensor()
{
    delay(5000);
    float acc = 0.0; //Accumulator

    Serial.println("Calibrating Sensors...");
    LCD_Print("Calib...");
    RelaySetAll(1);
    Serial.println("Filling Rolling Average Buffers..");
    for (uint16_t i = 0; i < 50000; i++)
    {
        DataSampling();
    }
    Serial.println("Finding Current Offset..");
    delay(1000);
    for (uint16_t i = 0; i < 10000; i++)
    {
        acc += analogRead(CS_1);
        if (i % 100 == 0)
        {
            Serial.print(".");
        }
    }
    Serial.print("\n");
    CS_1_OS = (float)acc / 10000.0;
    LCD_Print("Complete");
    Serial.println("Calibration Complete.");
    Serial.println("Values Found: " + String(CS_1_OS));
}

/*
    Test Relays
*/
void RelaySetAll(uint8_t state)
{
    if (state == 1)
    {
        Serial.println("All Outlets Off");
        LCD_Print("All_OFF_");
    }
    else
    {
        Serial.println("All Outlets On");
        LCD_Print("All_ON__");
    }
    digitalWrite(DEV_1, state);
    digitalWrite(DEV_2, state);
    digitalWrite(DEV_3, state);
    digitalWrite(DEV_4, state);
    digitalWrite(DEV_5, state);
    digitalWrite(DEV_6, state);
    digitalWrite(DEV_7, state);
    digitalWrite(DEV_8, state);
}

int8_t getDevicePin(uint8_t device)
{
    //Pin Linked to Device Number
    String path = "/devices/dev/" + String(device) + ".txt";
    String hold_str = ReadData(path);

    return (int8_t)hold_str.toInt();
}

//------END
