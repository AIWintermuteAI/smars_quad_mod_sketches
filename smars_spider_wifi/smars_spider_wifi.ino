/*
2018-09-24
Modified by V.Palleschi from the original by Salvatore Fancello,
http://www.progettiarduino.com
e-mail: salvatorefancello89@gmail.com
e-mail: vpalleschi@gmail.com
*/
#define LONGDELAY 50
#define SHORTDELAY 10
#define SERVOMIN 0
#define SERVOMAX 180

#include <Servo.h>
#include <WiFiNINA.h>

Servo back_left_knee;
Servo back_left_hip;

Servo back_right_knee;
Servo back_right_hip;

Servo front_left_knee;
Servo front_left_hip;

Servo front_right_knee;
Servo front_right_hip;

char ssid[] = "smars_spider";
char pass[] = "";
int status = WL_IDLE_STATUS;

IPAddress ipAddress = IPAddress(192, 168, 4, 2); // Smartphone ip
#define PORT 50123 //Change PORT if you need, alsop you have to set the same port in the mobile app

// Initialize the client library
WiFiClient client;

String numericPart = "";
char codeReceived;
char lastCodeReceived = ' ';

void setup()
{
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }
  
    Serial.begin(115200);
    front_left_knee.attach(A0);
    front_left_hip.attach(A1);
    back_left_knee.attach(A2);
    back_left_hip.attach(A3);

    front_right_knee.attach(13);
    front_right_hip.attach(12);
    back_right_knee.attach(10);
    back_right_hip.attach(9);
    //prepare_jump();
    stand();
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
   Serial.print("Moving  joint: ");
   Serial.println(number);
   switch (number) {
    case 0:
        front_left_hip.write(angle);
        Serial.print("front_left_hip");
        Serial.println(angle);
        break;
    case 1:
        front_left_knee.write(angle);
        Serial.print("front_left_knee");
        Serial.println(angle);
        break;
    case 2:
        back_left_hip.write(angle);
        Serial.print("back_left_hip");
        Serial.println(angle);
        break;
    case 3:
        back_left_knee.write(angle);
        Serial.print("back_left_knee");
        Serial.println(angle);
        break;
    case 4:
        front_right_hip.write(angle);
        Serial.print("front_right_hip");
        Serial.println(angle);
        break;
    case 5:
        front_right_knee.write(angle);
        Serial.print("front_right_knee");
        Serial.println(angle);
        break;
    case 6:
        back_right_hip.write(angle);
        Serial.print("back_right_hip");
        Serial.println(angle);
        break;
    case 7:
        back_right_knee.write(angle);
        Serial.print("back_right_knee");
        Serial.println(angle);
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

void moveleg(uint8_t number, int8_t ankle, int8_t hip)
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
    mov(4, 150);
    delay(LONGDELAY);
    mov(3, 150);
    delay(LONGDELAY);
    mov(1, 90);

    delay(LONGDELAY);
    stp(3, 30);

    delay(LONGDELAY);
    stp(4, 30);

    delay(LONGDELAY);
    mov(4, 90);
    delay(LONGDELAY);
    mov(2, 150);
    delay(LONGDELAY);
    mov(1, 150);
    delay(LONGDELAY);
    mov(3, 90);
    delay(LONGDELAY);
    stp(1, 30);
    stand();
}

void backward()
{
    stp(4, 30);
    delay(LONGDELAY);
    stp(3, 30);

    delay(LONGDELAY);
    mov(3, 90);
    delay(LONGDELAY);
    mov(1, 30);
    delay(LONGDELAY);
    mov(2, 30);
    delay(LONGDELAY);
    mov(4, 90);

    delay(LONGDELAY);
    stp(2, 150);

    delay(LONGDELAY);
    stp(1, 150);

    delay(LONGDELAY);
    mov(1, 90);
    delay(LONGDELAY);
    mov(2, 90);
    delay(LONGDELAY);
    mov(3, 30);
    delay(LONGDELAY);
    mov(4, 30);

    delay(LONGDELAY);
    stp(4, 150);
    delay(LONGDELAY);
    stand();
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

void connect() {
    if (!client.connect(ipAddress, PORT)) {
        Serial.println("Connection to host failed");
        delay(1000);
        return;
    }
    Serial.print("Connected to: ");
    Serial.println(client.remoteIP().toString());

}

void loop()
{
    // compare the previous status to the current status
    while (WL_AP_CONNECTED != WiFi.status()) {
      // it has changed update the variable
      status = WiFi.status();
    
      if (status == WL_AP_CONNECTED) {
        // a device has connected to the AP
        Serial.println("Device connected to AP");
      } else {
        // a device has disconnected from the AP, and we are back in listening mode
        Serial.println("Device disconnected from AP");
      }

      connect();
    }
    
    while (client.available() > 0) {
      String line = client.readStringUntil('\n');
      numericPart = "";
      for (int i = 0; i < line.length(); i++) {
          int character = line[i];
          if (isDigit(character)) {
              numericPart += (char) character;
          } else if (character != '\n') {
              codeReceived = character;
          } else {
              break;
          }
      }
    }
    
    if (lastCodeReceived == codeReceived) {
      return;
    }
    lastCodeReceived = codeReceived;
    Serial.println(codeReceived);
    
    switch (codeReceived) {
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
        //default:
        //    Serial.println("Invalid Command");
        //    break;
    }
}
