/*
2018-09-24
Modified by V.Palleschi from the original by Salvatore Fancello,
http://www.progettiarduino.com
2024-08-22
Modified by D.Maslov
*/
#define LONGDELAY 50
#define SHORTDELAY 10
#define SERVOMIN 0
#define SERVOMAX 180

#define MIN_PULSE 540
#define MAX_PULSE 2400

#include <Servo.h>

#ifndef ARDUINO_ARCH_MBED_NANO
  #include <EEPROM.h>
#endif

Servo back_left_knee;
Servo back_left_hip;

Servo back_right_knee;
Servo back_right_hip;

Servo front_left_knee;
Servo front_left_hip;

Servo front_right_knee;
Servo front_right_hip;

int32_t offset_array[] = { 0, 0, 0, 0,
                            0, 0, 0, 0 };

void setup()
{
    Serial.begin(115200);
#ifndef ARDUINO_ARCH_MBED_NANO
    front_left_hip.attach(A1, MIN_PULSE, MAX_PULSE);
    front_left_knee.attach(A0, MIN_PULSE, MAX_PULSE);
    back_left_hip.attach(A3, MIN_PULSE, MAX_PULSE);
    back_left_knee.attach(A2, MIN_PULSE, MAX_PULSE);

    front_right_hip.attach(D4, MIN_PULSE, MAX_PULSE);
    front_right_knee.attach(D5, MIN_PULSE, MAX_PULSE);
    back_right_hip.attach(D2, MIN_PULSE, MAX_PULSE);
    back_right_knee.attach(D3, MIN_PULSE, MAX_PULSE);
#else
    front_left_hip.attach(A1);
    front_left_knee.attach(A0);
    back_left_hip.attach(A3);
    back_left_knee.attach(A2);

    front_right_hip.attach(4);
    front_right_knee.attach(5);
    back_right_hip.attach(2);
    back_right_knee.attach(3);
#endif

#ifndef ARDUINO_ARCH_MBED_NANO
    EEPROM.begin(512);
    for (uint8_t i = 0; i < 8; i++) {
      EEPROM.get(i * 4, offset_array[i]);
      Serial.print(offset_array[i]);
      Serial.print(' ');
    }
#endif
    init_state();
    //stand();
    //test_servos();
    //test_servos_2();
}

void servocontrol(uint8_t number, uint8_t angle)
{
    // hip
    if (number==2 || number==0) //LEFT
    {
        angle = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    }
    if (number==6 || number==4) //RIGHT
    {
        angle = map(angle, 180, 0, SERVOMIN, SERVOMAX);
    }

    // knees
    if (number==7 || number==1) //LEFT
    {
        angle = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    }
    if (number==3 ||number==5) // RIGHT
    {
        angle = map(angle, 180, 0, SERVOMIN, SERVOMAX);
    }
    Serial.print("Moving  joint ");
    Serial.print(number);
    Serial.print(" to angle ");
    Serial.print(angle);
    Serial.print(" with offset ");
    Serial.print(offset_array[number]);

   uint8_t angle_with_offset = angle + offset_array[number];
   switch (number) {
    case 0:
        front_left_hip.write(angle_with_offset);
        Serial.println(" front_left_hip");
        break;
    case 1:
        front_left_knee.write(angle_with_offset);
        Serial.println(" front_left_knee");
        break;
    case 2:
        back_left_hip.write(angle_with_offset);
        Serial.println(" back_left_hip");
        break;
    case 3:
        back_left_knee.write(angle_with_offset);
        Serial.println(" back_left_knee");
        break;
    case 4:
        front_right_hip.write(angle_with_offset);
        Serial.println(" front_right_hip");
        break;
    case 5:
        front_right_knee.write(angle_with_offset);
        Serial.println(" front_right_knee");
        break;
    case 6:
        back_right_hip.write(angle_with_offset);
        Serial.println(" back_right_hip");
        break;
    case 7:
        back_right_knee.write(angle_with_offset);
        Serial.println(" back_right_knee");
        break;
    default:
        break;
    }
}

void test_servos()
{
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j <= 120; j++)
        {
            servocontrol(i, j);
            delay(50);
        }
        for (int j = 120; j >= 0; j--)
        {
            servocontrol(i, j);
            delay(50);
        }
    }
}

// test all servos using mov() function
void test_servos_2() {
    for (int i = 1; i < 5; i++) {
        for (int j = 0; j <= 120; j++) {
            mov(i, j);
            delay(5);
        }
        u(i);
        delay(250);
        for (int j = 120; j >= 0; j--) {
            mov(i, j);
            delay(5);
        }
        d(i);
        delay(250);
    }
}

void moveleg(uint8_t number, uint8_t ankle, uint8_t hip)
{
    if (ankle >= 0)
    {
        servocontrol(2*(number-1), ankle);
    };
    if (hip >= 0)
    {
        servocontrol(2*(number-1)+1, hip);
    };
}

void u(int n)
{
    servocontrol(2*(n-1)+1, 90);
}

void d(int n)
{
    servocontrol(2*(n-1)+1, 60);
}

void stp(int n, int a)
{
    yield();
    u(n);
    delay(LONGDELAY);
    mov(n, a);
    delay(LONGDELAY);
    d(n);
}

void mov(int n, int a)
{
    servocontrol(2*(n-1), a);
}

void init_state()
{
    yield();
    moveleg(1, 90, 90);
    moveleg(2, 90, 90);
    moveleg(3, 90, 90);
    moveleg(4, 90, 90);
}

void forward()
{
    stp(1, 30);
    delay(LONGDELAY);
    stp(2, 30);
    delay(LONGDELAY);
    mov(2, 90);
    delay(LONGDELAY);
    mov(4, 120);
    delay(LONGDELAY);
    mov(3, 110);
    delay(LONGDELAY);
    mov(1, 90);

    delay(LONGDELAY);
    stp(3, 30);

    delay(LONGDELAY);
    stp(4, 30);

    delay(LONGDELAY);
    mov(4, 90);
    delay(LONGDELAY);
    mov(2, 120);
    delay(LONGDELAY);
    mov(1, 160);
    delay(LONGDELAY);
    mov(3, 95);
    delay(LONGDELAY);
    stp(1, 30);
    delay(LONGDELAY);
    //stand();
}

void backward()
{
    stp(1, 160);
    delay(LONGDELAY);
    stp(2, 160);
    delay(LONGDELAY);
    mov(2, 90);
    delay(LONGDELAY);
    mov(4, 70);
    delay(LONGDELAY);
    mov(3, 70);
    delay(LONGDELAY);
    mov(1, 90);

    delay(LONGDELAY);
    stp(3, 120);

    delay(LONGDELAY);
    stp(4, 120);

    delay(LONGDELAY);
    mov(4, 90);
    delay(LONGDELAY);
    mov(2, 20);
    delay(LONGDELAY);
    mov(1, 20);
    delay(LONGDELAY);
    mov(3, 90);
    delay(LONGDELAY);
    stp(1, 160);
    delay(LONGDELAY);
    //stand();
}

void rotate_r()
{
    stp(3, 150);
    delay(LONGDELAY);
    stp(1, 30);
    delay(LONGDELAY);
    stp(2, 30);
    delay(LONGDELAY);
    stp(4, 150);
    delay(LONGDELAY);
    mov(4, 90);
    mov(2, 90);
    mov(1, 90);
    mov(3, 90);
    stand();
}

void rotate_l()
{
    stp(3, 30);
    delay(LONGDELAY);
    stp(4, 30);
    delay(LONGDELAY);
    stp(2, 150);
    delay(LONGDELAY);
    stp(1, 150);
    delay(LONGDELAY);
    mov(3, 90);
    mov(1, 90);
    mov(2, 90);
    mov(4, 90);
    stand();
}

void stand()
{
    moveleg(1, 90, 60);
    moveleg(2, 90, 60);
    moveleg(3, 90, 60);
    moveleg(4, 90, 60);
}

void squat()
{
  for (int i = 30; i <= 90; i++)
  {
      moveleg(1, 90, i);
      moveleg(2, 90, i);
      moveleg(3, 90, i);
      moveleg(4, 90, i);
      delay(LONGDELAY);
  }
}

void wave_l()
{
    mov(2, 60);
    delay(LONGDELAY);
    mov(4, 60);
    delay(LONGDELAY);
    mov(1, 140);
    delay(LONGDELAY);
    mov(3, 60);
    delay(LONGDELAY);
    u(1);
    for (int j = 0; j < 4; j++)
    {
        for (int i = 140; i >= 30; i--)
        {
            delay(SHORTDELAY);
            mov(1, i);
        }
        for (int i = 30; i <= 140; i++)
        {
            delay(SHORTDELAY);
            mov(1, i);
        }
    }
    stand();
}

void wave_r()
{
    mov(2, 60);
    delay(LONGDELAY);
    mov(4, 60);
    delay(LONGDELAY);
    mov(1, 60);
    delay(LONGDELAY);
    mov(3, 140);
    delay(LONGDELAY);
    u(3);
    for (int j = 0; j < 4; j++)
    {
        for (int i = 140; i >= 30; i--)
        {
            delay(SHORTDELAY);
            mov(3, i);
        }
        for (int i = 30; i <= 140; i++)
        {
            delay(SHORTDELAY);
            mov(3, i);
        }
    }
    stand();
}

void skew_l()
{
    mov(3, 150);
    delay(LONGDELAY);
    mov(1, 30);
    delay(LONGDELAY);
    mov(2, 30);
    delay(LONGDELAY);
    mov(4, 150);
    delay(LONGDELAY);
}

void skew_r()
{
    mov(3, 30);
    delay(LONGDELAY);
    mov(1, 150);
    delay(LONGDELAY);
    mov(2, 150);
    delay(LONGDELAY);
    mov(4, 30);
    delay(LONGDELAY);
}

void courtsy()
{
    u(1);
    u(3);
    moveleg(2, -1, 60);
    moveleg(4, -1, 60);
}

void prepare_jump()
{

    u(1);
    u(2);
    u(3);
    u(4);
    delay(LONGDELAY*4);
    moveleg(1, -1, 30);
    moveleg(2, -1, 30);
    moveleg(3, -1, 30);
    moveleg(4, -1, 30);
    delay(LONGDELAY*4);
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void calibration()
{
#ifndef ARDUINO_ARCH_MBED_NANO
    bool completed = false;
    for (uint8_t i = 0; i < 8; i++) {
      EEPROM.get(i * 4, offset_array[i]);
      Serial.print(offset_array[i]);
      Serial.print(' ');
    }
    while (!completed) {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Serial.println(input);
        if (input == "done") {
          completed = true;
          break;
        }
        if (input == "reset") {
          completed = true;
          for (uint8_t i = 0; i < 8; i++) {
            offset_array[i] = 0;
          }
          break;
        }
        String servo_num = getValue(input, ' ', 0);
        String servo_offset = getValue(input, ' ', 1);
        offset_array[servo_num.toInt()] = servo_offset.toInt();
        init_state();
      }
    }
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(offset_array[i]);
      Serial.print(' ');
      EEPROM.put(i * 4, offset_array[i]);
    }
    Serial.println(' ');
    EEPROM.commit();
    init_state();
#endif
}

void loop()
{
    //forward();

    if (Serial.available() > 0) {
        char input = Serial.read();
        Serial.println(input);
        switch (input) {
            case 'F':
                forward();
                Serial.println("Forward");
                break;
            case 'B':
                backward();
                Serial.println("Backward");
                break;
            case 'R':
                rotate_r();
                Serial.println("Rotate Right");
                break;
            case 'L':
                rotate_l();
                Serial.println("Rotate Left");
                break;
            case 'S':
                stand();
                Serial.println("Stand");
                break;
            case 'Q':
                squat();
                Serial.println("Squat");
                break;
            case 'W':
                wave_l();
                Serial.println("Wave Left");
                break;
            case 'w':
                wave_r();
                Serial.println("Wave Right");
                break;
            case 'K':
                skew_l();
                Serial.println("Skew Left");
                break;
            case 'k':
                skew_r();
                Serial.println("Skew Right");
                break;
            case 'C':
                courtsy();
                Serial.println("Courtsy");
                break;
            case 'J':
                prepare_jump();
                Serial.println("Prepare Jump");
                break;
            case 'i':
                init_state();
                Serial.println("Initial state");
                break;
            case 'c':
                Serial.println("Calibration mode");
                calibration();
                break;
            case '\n':
            case '\r':
                break;
            default:
                Serial.println("Invalid command");
                break;
        }
    }
}
