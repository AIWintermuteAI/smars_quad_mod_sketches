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

#define MIN_PULSE 540
#define MAX_PULSE 2400

#include <Servo.h>
#include "pico/multicore.h"

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

int32_t offset_array[] = { 0, 0, 3, 5,
                            -2, 10, -5, 0 };


/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW    2
#define EI_CLASSIFIER_SLICE_SIZE (EI_CLASSIFIER_RAW_SAMPLE_COUNT / EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/*
 ** NOTE: If you are seeing Error sample buffer overrun.
 **
 ** Cortex M0+ has no hardware floating point support, therfore DSP
 ** operations are rather slow. You can try decreasing
 ** EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW or tweak the MFCC/MFE block parameters
 ** in Studio and re-deploy your project to get faster processing times.
 */

/* Includes ---------------------------------------------------------------- */
#include <robot-control-english_inferencing.h>
#include <PDM.h>

/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static volatile bool record_ready = false;
// static signed short *sampleBuffer;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

void setup_servos()
{
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
}

void disable_servos()
{
    front_left_hip.detach();
    front_left_knee.detach();
    back_left_hip.detach();
    back_left_knee.detach();;

    front_right_hip.detach();
    front_right_knee.detach();
    back_right_hip.detach();
    back_right_knee.detach();
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

void control_loop()
{
  setup_servos();
  #ifndef ARDUINO_ARCH_MBED_NANO
    EEPROM.begin(512);
    for (uint8_t i = 0; i < 8; i++) {
      EEPROM.get(i * 4, offset_array[i]);
      Serial.print(offset_array[i]);
      Serial.print(' ');
    }
#endif
    //init_state();
    stand();
  std::map<int, char> mapping;
  mapping[0] = 'B';
  mapping[1] = 'F';
  mapping[2] = 'L';
  mapping[3] = 'R';
  while (true) {
    //stand();
    uint32_t input = multicore_fifo_pop_blocking();
    setup_servos();
    char lookup = mapping[input];
    Serial.println(lookup);
    switch (lookup) {
        case 'F':
            forward();
            forward();
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
        default:
            Serial.println("Invalid Command");
            break;
    }
    disable_servos();
  }
}

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    run_classifier_init();
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    multicore_launch_core1(control_loop);
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR res = run_classifier_continuous(&signal, &result, debug_nn);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

    if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
        // print inference return code
        ei_printf("run_classifier returned: %d\r\n", res);
        print_inference_result(result);
        print_results = 0;
    }
}

/**
 * @brief      PDM buffer full callback
 *             Copy audio data to app buffers
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if ((inference.buf_ready == 0) && (record_ready == true)) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

            if (inference.buf_count >= inference.n_samples) {
                inference.buf_select ^= 1;
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    PDM.setBufferSize(2048);
    delay(250);

    // initialize PDM with:
    // - one channel (mono mode)
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("ERR: Failed to start PDM!");
        return false;
    }

    // optionally set the gain, defaults to 24
    // Note: values >=52 not supported
    //PDM.setGain(40);

    record_ready = true;

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW is currently set to %d\n", EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
        ret = false;
    }

    while (inference.buf_ready == 0) {
        delay(1);
    }

    inference.buf_ready = 0;

    return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    record_ready = false;
}

void print_inference_result(ei_impulse_result_t result) {
    uint8_t top_result = 255;
    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
        if (result.classification[i].value == 1) top_result = i;
    }
    if (top_result != 255) {
      microphone_inference_end();
      multicore_fifo_push_blocking(top_result);
      delay(4000);
      if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
      return;
    }
    }
}
