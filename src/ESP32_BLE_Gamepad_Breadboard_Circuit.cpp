/*
 * ANDROID GAMEPAD
 * {A=1, B=2, C=3, X=4, Y=5, Z=6, L1=7, R1=8, L2=9, R2=10,
 * Select=11, Start=12, PS=13, L3=14 , R3=15}
 *
 * PS GAMEPAD MODE
 * {SQUARE=1, X=2, CIRCLE=3, TRIANGLE=4, L1=5, R1=6, L2=7, R2=8,
 * Select=9, Start=10, L3=11, R3=12, PS=13}
 *
 */

#include <Arduino.h>

#include <BleConnectionStatus.h>
#include <BleGamepad.h>
#include <BleGamepadConfiguration.h>
#include <BleOutputReceiver.h>

#include <BleGamepad.h>
// #include "sbus.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Defines.h"
#include "Sbus_Rx.h"


//SBUS x8r(Serial2);
//// bfs::SbusRx sbus_rx(&Serial2, 8, 9, true, false);
//// bfs::SbusData data;

uint16_t channels[16];
bool failSafe;
bool lostFrame;

#define SBUS_ON_PIN 1     // LOW indicates sbus, HIGH indicates joystick
#define ESPNOW_MODE_PIN 3 // LOW indicates esp-now, HIGH indicates bluetooth

volatile byte buttonReleased = false;
bool isDrone = true; // airplane false, drone true
void buttonReleasedInterrupt() {
  buttonReleased = true;
}

// ABXY BUTTONS
#define X_BUTTON 23        // A
#define CIRCLE_BUTTON 22   // B
#define TRIANGLE_BUTTON 21 // Y
#define SQUARE_BUTTON 19   // X

// TRIGGERS
#define R1_BUTTON 0
#define R2_BUTTON 0
#define L1_BUTTON 0
#define L2_BUTTON 0

// MENU BUTTONS
#define START_BUTTON 0
#define SELECT_BUTTON 1
#define PS_BUTTON 0

// JOYSTICKS BUTTONS
#define R3_BUTTON 0
#define L3_BUTTON 0
#define BUTTON_RIGHT_JY_PIN 9

// JOYSTICKS
#define LEFT_VRX_JOYSTICK 4
#define LEFT_VRY_JOYSTICK 5
#define RIGHT_VRX_JOYSTICK 6
#define RIGHT_VRY_JOYSTICK 7

#define NUM_BUTTONS 13

// The order of these three arrays matters a lot, be carefully when changing them
int buttonsPins[NUM_BUTTONS] = {X_BUTTON, CIRCLE_BUTTON, TRIANGLE_BUTTON, SQUARE_BUTTON,
                                R1_BUTTON, R2_BUTTON, L1_BUTTON, L2_BUTTON,
                                START_BUTTON, SELECT_BUTTON, PS_BUTTON,
                                R3_BUTTON, L3_BUTTON};

// There is not buttons for Y and Z
int androidGamepadButtons[NUM_BUTTONS] = {1, 2, 3, 4, 8, 10, 7, 9, 12, 11, 13, 15, 14};
int PS1GamepadButtons[NUM_BUTTONS] = {2, 3, 4, 1, 6, 8, 5, 7, 10, 9, 13, 12, 11};
int PCGamepadButtons[NUM_BUTTONS] = {1, 2, 4, 3, 6, 8, 5, 7, 10, 9, 0, 12, 11};

uint16_t leftVrxJoystickLecture = 0;
uint16_t leftVryJoystickLecture = 0;
uint16_t rightVrxJoystickLecture = 0;
uint16_t rightVryJoystickLecture = 0;

uint16_t leftVrxJoystickValue = 0;
uint16_t leftVryJoystickValue = 0;
uint16_t rightVrxJoystickValue = 0;
uint16_t rightVryJoystickValue = 0;

typedef enum
{
  ANDROID,
  PS1,
  PC
} GamepadModes;
GamepadModes gamepadMode = ANDROID;

BleGamepad bleGamepad("Maker101 Gamepad", "Maker101 Home");
BleGamepadConfiguration bleGamepadConfig;

void joysticksHandlerForMobile(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry)
{
  bleGamepad.setLeftThumb(leftVrx, leftVryJoystickValue);
  bleGamepad.setRightThumb(rightVrxJoystickValue, rightVryJoystickValue);
}

void joysticksHandlerForPC(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry)
{
  bleGamepad.setX(leftVrxJoystickValue);
  bleGamepad.setY(leftVryJoystickValue);
  bleGamepad.setZ(rightVrxJoystickValue);
  bleGamepad.setRX(rightVryJoystickValue);
}

/////////////////  esp now start ///////////
uint8_t broadcastAddressAirplane[] = {0x54, 0x32, 0x04, 0x3d, 0xd4, 0x0c}; // Airplane
uint8_t broadcastAddressDrone[] = {0x9c, 0x9e, 0x6e, 0x43, 0x84, 0xe4}; // drone
uint8_t broadcastAddress[]  = {0x9c, 0x9e, 0x6e, 0x43, 0x84, 0xe4};
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  uint16_t throttle;
  uint16_t rudder;
  uint16_t elevator;
  uint16_t aileron;
  uint16_t mode;
  uint16_t panic;
} struct_message;

// Create a struct_message called myData
struct_message espNowData;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/////////////////  esp now end ///////////
void setup()
{
  // put your setup code here, to run once:
  delay(500);
  Serial.begin(115200);
  pinMode(1, INPUT_PULLUP);
  pinMode(ESPNOW_MODE_PIN, INPUT_PULLUP);
  // x8r.begin(8, 9, true, 100000);
  initSbusRx();
  pinMode(BUTTON_RIGHT_JY_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(BUTTON_RIGHT_JY_PIN),
  //                 buttonReleasedInterrupt,
  //                 FALLING);
  // sbus_rx.Begin();

  // for (int i = 0; i < NUM_BUTTONS; i++)
  // {
  //   pinMode(buttonsPins[i], INPUT_PULLUP);
  // }
  int transmitMode = digitalRead(ESPNOW_MODE_PIN);
  if (transmitMode == LOW)
  {
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer drone");
      return;
    }
    memcpy(peerInfo.peer_addr, broadcastAddressAirplane, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer airplane");
      return;
    }
  }
  else
  {
    bleGamepadConfig.setAutoReport(false);
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
    bleGamepadConfig.setVid(0xe502);
    bleGamepadConfig.setPid(0xabcd);
    bleGamepadConfig.setHatSwitchCount(4);
    bleGamepad.begin(&bleGamepadConfig);
  }
}

void loop()
{
  // printf("loop\n");
  
  getSbus();
  delay(30);

  if (buttonReleased) {
    buttonReleased = false;
    if (isDrone)
    {
      isDrone = false; 
      digitalWrite(LED_BUILTIN, HIGH);
      printf("Airplane mode\n");
      // neopixelWriter.setPixelColor(100, 0, 0, 0);
    }
    else
    {
      isDrone = true;
      printf("Drone mode\n");
      digitalWrite(LED_BUILTIN, LOW);
      // neopixelWriter.setPixelColor(100, 100, 0, 0);
    }
  }
  Sbus_Data rxData = getSbusData();
    if (digitalRead(SBUS_ON_PIN) == LOW)  { // sbus on
      //// Plainflight Display the received data 
      for (uint8_t i = 0; i < 16; i++) 
      {
        printf("%d", rxData.ch[i]);
        printf("\t");
      }
      /// Display lost frames and failsafe data 
      printf("%d", rxData.lost_frame);
      printf("\t");
      printf("%d \n", rxData.failsafe);
    }
    if (!rxData.failsafe)
    {
      leftVrxJoystickLecture = rxData.ch[3];
      leftVryJoystickLecture = rxData.ch[0];
      rightVrxJoystickLecture = rxData.ch[1];
      rightVryJoystickLecture = rxData.ch[2];
      // map(long x, long in_min, long in_max, long out_min, long out_max)
      leftVrxJoystickValue = map(leftVrxJoystickLecture, 335, 1706, 0, 32737);
      leftVryJoystickValue = map(leftVryJoystickLecture, 335, 1706, 0, 32737);
      rightVrxJoystickValue = map(rightVrxJoystickLecture, 240, 1615, 0, 32737);
      rightVryJoystickValue = map(rightVryJoystickLecture, 280, 1650, 0, 32737);
    }
  //// end Plainflight
  // if (sbus_rx.Read())
  // {
  //   printf("loop\n");
  //   data = sbus_rx.data();
  //   for (int8_t i = 0; i < data.NUM_CH; i++)
  //   {
  //     printf("%d", data.ch[i]);
  //     printf("\t");
  //   }
  //   /* Display lost frames and failsafe data */
  //   printf("%d", data.lost_frame);
  //   printf("\t");
  //   printf("%d\n", data.failsafe);
  // }
  ////// end sbus_rx 

  /* old working code
  if (x8r.read(&channels[0], &failSafe, &lostFrame))
  {
    printf("   ");
    for (int8_t i = 15; i >= 10; i--)
    {
      printf(" %u", channels[i]);
    }
    printf("\n");

    leftVrxJoystickLecture = channels[13];
    leftVryJoystickLecture = channels[10];
    rightVrxJoystickLecture = channels[11];
    rightVryJoystickLecture = channels[12];
    // map(long x, long in_min, long in_max, long out_min, long out_max)
    leftVrxJoystickValue = map(leftVrxJoystickLecture, 340, 1706, 0, 32737);
    leftVryJoystickValue = map(leftVryJoystickLecture, 340, 1706, 0, 32737);
    rightVrxJoystickValue = map(rightVrxJoystickLecture, 240, 1615, 0, 32737);
    rightVryJoystickValue = map(rightVryJoystickLecture, 280, 1650, 0, 32737);
    // neopixelWriter.setPixelColor(0, leftVrxJoystickLecture, 0, 0);
  }*/

  // if (x8r.read(&channels[1], &failSafe, &lostFrame))
  // {
  //   printf("2Setting duty cycle to %d%% %d %d\n", channels[1], failSafe, lostFrame);
  //   leftVryJoystickLecture = channels[1];
  // }

  // if (x8r.read(&channels[2], &failSafe, &lostFrame))
  // {
  //   printf("3Setting duty cycle to %d%% %d %d\n", channels[2], failSafe, lostFrame);
  //   rightVrxJoystickLecture = channels[2];
  // }

  // if (x8r.read(&channels[3], &failSafe, &lostFrame))
  // {
  //   printf("4Setting duty cycle to %d%% %d %d\n", channels[3], failSafe, lostFrame);
  //   rightVryJoystickLecture = channels[3];
  // }

  // put your main code here, to run repeatedly:
  if (digitalRead(SBUS_ON_PIN) == HIGH) // high joystick
  {
    // Joysticks lecture
    leftVrxJoystickLecture = analogRead(LEFT_VRX_JOYSTICK);
    leftVryJoystickLecture = analogRead(LEFT_VRY_JOYSTICK);
    rightVrxJoystickLecture = analogRead(RIGHT_VRX_JOYSTICK);
    rightVryJoystickLecture = analogRead(RIGHT_VRY_JOYSTICK);
    printf("Joystick values to %d %d %d %d \n", leftVrxJoystickLecture, leftVryJoystickLecture, rightVrxJoystickLecture, rightVryJoystickLecture);
    // Compute joysticks value
    leftVrxJoystickValue = map(leftVrxJoystickLecture, 4095, 0, 0, 32737);
    leftVryJoystickValue = map(leftVryJoystickLecture, 4095, 0 , 0, 32737);
    rightVrxJoystickValue = map(rightVrxJoystickLecture, 4095, 0, 0, 32737);
    rightVryJoystickValue = map(rightVryJoystickLecture, 4095, 0, 0, 32737);
  }

  if (bleGamepad.isConnected())
  {
    switch (gamepadMode)
    {
    case ANDROID:
      for (int i = 0; i < NUM_BUTTONS; i++)
      {
        if (!digitalRead(buttonsPins[i]))
        {
          bleGamepad.press(androidGamepadButtons[i]);
        }
        else
        {
          bleGamepad.release(androidGamepadButtons[i]);
        }
        joysticksHandlerForMobile(leftVrxJoystickValue, leftVryJoystickValue, rightVrxJoystickValue, rightVryJoystickValue);
      }
      break;

    case PS1:
      for (int i = 0; i < NUM_BUTTONS; i++)
      {
        if (!digitalRead(buttonsPins[i]))
        {
          bleGamepad.press(PS1GamepadButtons[i]);
        }
        else
        {
          bleGamepad.release(PS1GamepadButtons[i]);
        }
        joysticksHandlerForMobile(leftVrxJoystickValue, leftVryJoystickValue, rightVrxJoystickValue, rightVryJoystickValue);
      }
      break;

    case PC:
      for (int i = 0; i < NUM_BUTTONS; i++)
      {
        if (!digitalRead(buttonsPins[i]))
        {
          bleGamepad.press(PCGamepadButtons[i]);
        }
        else
        {
          bleGamepad.release(PCGamepadButtons[i]);
        }
        joysticksHandlerForPC(leftVrxJoystickValue, leftVryJoystickValue, rightVrxJoystickValue, rightVryJoystickValue);
      }
      break;
    }

    bleGamepad.sendReport();
  }

  if (digitalRead(ESPNOW_MODE_PIN) == LOW)
  {

    espNowData.throttle = leftVryJoystickValue;
    espNowData.rudder = leftVrxJoystickValue;
    espNowData.elevator = rightVryJoystickValue;
    espNowData.aileron = rightVrxJoystickValue;
    espNowData.mode = 0;
    espNowData.panic = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&espNowData, sizeof(espNowData));
    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
  }
}
