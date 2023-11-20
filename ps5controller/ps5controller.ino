#include <PS5BT.h>
#include <usbhub.h>
#include <math.h>
#include <SoftwareSerial.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb);  // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS5BT class in two ways */
// This will start an inquiry and then pair with the PS5 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS5 controller will then start to blink rapidly indicating that it is in pairing mode
//PS5BT PS5(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
PS5BT PS5(&Btd);

SoftwareSerial coolSerial(2, 3);
SoftwareSerial coolSerial2(4, 5);
char n[9];

bool printAngle = false, printTouch = false, ifDecay = false;
uint16_t lastMessageCounter = -1;
uint8_t player_led_mask = 0;
bool microphone_led = false;
volatile uint8_t Checksum_Calc;

char brake = 'B';
double speed_wagon = 0, initial_sw = 0, initTime = 0;
double theta = 0;

void getByte(char mode, String speed, String angle) {
  n[0] = mode;
  for (int i = 1; i < 5; i++) {
    if (speed[i - 1] != NULL) {
      n[i] = speed[i - 1];
    } else {
      n[i] = 'x';
    }
  }
  for (int i = 5; i < 9; i++) {
    if (angle[i - 5] != NULL) {
      n[i] = angle[i - 5];
    } else {
      n[i] = 'x';
    }
  }
}

void setup() {
#if !defined(__MIPSEL__)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  // Halt
  }
  Serial.print(F("\r\nPS5 Bluetooth Library Started"));
  coolSerial.begin(115200);
  coolSerial2.begin(500000);
  Serial.begin(57600);
}

void loop() {
  Usb.Task();
  if (PS5.connected() && lastMessageCounter != PS5.getMessageCounter()) {
    lastMessageCounter = PS5.getMessageCounter();

    if (abs(speed_wagon) >= 0.1) {
      if (brake != 'B' && brake != 'c' && brake != 'w' && brake != 'W' && brake != 'd' && brake != 'D') {
        brake = 'a';
      }
    }

    // turning angle
    if (!PS5.getButtonPress(R1) && !PS5.getButtonPress(L1)) {
      if (abs(PS5.getAnalogHat(RightHatX) - 127) > 10) {
        theta = PS5.getAnalogHat(RightHatX) - 127;
      } else {
        theta = 0;
      }
    } else {
      if (PS5.getButtonPress(R1) && PS5.getButtonPress(L1)) {
        theta = 0;
      } else if (PS5.getButtonPress(R1)) {
        theta = 127;
      } else if (PS5.getButtonPress(L1)) {
        theta = -127;
      }
    }
    // speed
    if (PS5.getButtonPress(L2) || PS5.getButtonPress(R2)) {
      if (PS5.getButtonPress(L2)) {
        speed_wagon -= constrain(pow(1.05, (millis() - initTime) / 1000) - 0.7, 0, 500);
      }
      if (PS5.getButtonPress(R2)) {
        speed_wagon += constrain(pow(1.1, (millis() - initTime) / 1000) - 0.8, 0, 500);
      }
    } else {
      initial_sw = speed_wagon;
      initTime = millis();
    }

    if (abs(PS5.getAnalogHat(LeftHatY) - 127) > 10) {
      speed_wagon += (127 - PS5.getAnalogHat(LeftHatY)) / 1000.0;
    } else if (ifDecay) {
      speed_wagon *= 0.995;
    }
    // ebs
    if (PS5.getButtonClick(TRIANGLE) || PS5.getButtonClick(CIRCLE) || PS5.getButtonClick(CROSS) || PS5.getButtonClick(SQUARE) || PS5.getButtonClick(UP) || PS5.getButtonClick(RIGHT) || PS5.getButtonClick(DOWN) || PS5.getButtonClick(LEFT) || PS5.getButtonClick(L3) || PS5.getButtonClick(R3)) {
      speed_wagon = 0;
      if (brake != 'B' && brake != 'c' && brake != 'w' && brake != 'W') {
        brake = 'b';
      }
      Serial.println("brake");
    }
    if (PS5.getButtonClick(OPTIONS)) {
      if (brake == 'a' || brake == 'b' || brake == 'd' || brake == 'D') {
        brake = 'B';
      } else if (brake == 'B' || brake == 'c' || brake == 'w' || brake == 'W') {
        brake = 'b';
        speed_wagon = 0;
        theta = 0;
      }
    }
    if (PS5.getButtonClick(CREATE)) {
      speed_wagon = 0;
      theta = 0;
      brake = 'c';
    }
    // toggle training wheels
    if (PS5.getButtonClick(MICROPHONE)) {
      Serial.println(F("\nTraining wheels!"));

      if (microphone_led == false) {
        if (brake == 'B' || brake == 'c' || brake == 'w') {
          microphone_led = !microphone_led;
          PS5.setMicLed(microphone_led);
          brake = 'W';
        }
        if (brake == 'a' || brake == 'b' || brake == 'd')
        {
          microphone_led = !microphone_led;
          PS5.setMicLed(microphone_led);
          brake = 'D';
        }
      } else {
        if (brake == 'B' || brake == 'c' || brake == 'W') {
          microphone_led = !microphone_led;
          PS5.setMicLed(microphone_led);
          brake = 'w';
        }
        if (brake == 'a' || brake == 'b' || brake == 'D')
        {
          microphone_led = !microphone_led;
          PS5.setMicLed(microphone_led);
          brake = 'd';
        }
      }
    }

    // horn
    if (PS5.getButtonClick(TOUCHPAD)) {
      Serial.print(F("\r\nTouchpad"));
      printTouch = !printTouch;
    }
    speed_wagon = constrain(speed_wagon, -30, 40);
    theta = constrain(theta, -255, 255);
    getByte(brake, String(int(speed_wagon)), String(int(theta)));
    Serial.print(n);
    Serial.print("  ");
    Serial.print(speed_wagon);
    Serial.print("  ");
    Serial.println(theta);
    coolSerial.write(n);
  }
}
