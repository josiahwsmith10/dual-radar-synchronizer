
/* Dual Radar Synchronizer

   Author:  Josiah W. Smith
   Advisor: Murat Torlak
   Intern:  Benjamin Roy

   Notes:
    - Arduino String object is different than C++ std::string or C-strings, see docs: 
      https://www.arduino.cc/reference/en/language/variables/data-types/stringobject/
    

   Rev1 - 08/26/2021

   Changelog:
   - Rev1: built for single radar use case
*/

/*
TODO
*/

#include <Arduino.h>
#include <stdio.h>
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <string.h>

// Defs
#define PCNT_TEST_UNIT PCNT_UNIT_0  // PCNT unit to use
#define STEPPER_PIN 5               // Pulse train GPIO pin
#define CTRL_PIN 4                  // Control / direction GPIO pin. If none use constant: PCNT_PIN_NOT_USED
#define RADAR1_TRIGGER_PIN 13       // Pin to send radar 1 triggers
#define RADAR2_TRIGGER_PIN 14       // Pin to send radar 2 triggers
#define CNT_LIMIT 32000             // Limit on counter before reset to 0. Overflow is protected 
#define X_ALWAYS_OFFSET 10          // Offset to be used always in pulses

// Globals
bool isScanning = 0;          // Boolean whether or not a scan is in progress
bool isNext = 0;              // Boolean whether or not the next horizontal scan is ready
bool isRadar1 = 0;            // Boolean whether or not to use radar 1
bool isRadar2 = 0;            // Boolean whether or not to use radar 2
String incomingMessage = "";  // Incoming message Arduino String
char incomingStr[64];         // Incoming message C-string

xQueueHandle pcnt_evt_queue;  // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL;   // user's ISR service handle
portBASE_TYPE res;                          // Event receiver?
TaskHandle_t serialListener_handle = NULL;  // Task handle for the serialListenerTask
TaskHandle_t pulseListener_handle = NULL;   // Task handle for the pulseListenerTask
TaskHandle_t sendTrigger1_handle = NULL;    // Task handle for the sendTriggerTask for radar 1
TaskHandle_t sendTrigger2_handle = NULL;    // Task handle for the sendTriggerTask for radar 2

bool isDebug = 0;       // Boolean whether to print debugging helps
bool isDebugError = 0;  // Boolean whether to print fatal error debugging helps
bool isDebugWaiting = 0;// Boolean whether to print waiting debugging helps

typedef struct{
  int pulses_per_rev = 0;
  double xStep_mm = 0;
  double xStep_pulses = 0;
  double xOffset_mm = 0;
  double xOffset_pulses = 0;
  double DeltaX_mm = 0;
  int DeltaX_pulses = 0;
  int numX = 0;
  int numY = 0;
  int radarSelect = 1;
} scanParameters;

scanParameters scan;

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;
pcnt_evt_t evt; // PCNT event handler

// Prototypes
void serialListenerTask(void * parameters);
void pulseListenerTask(void * parameters);
void sendTriggerTask(void * parameters);

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * and timestamp to the main program using a queue.
 */
static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i = 0;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    
    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;

            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/* Initialize PCNT functions for one channel:
 *  - configure and initialize PCNT with pos-edge counting 
 *  - set up the input filter
 *  - set up the counter events to watch
 * Variables:
 * UNIT - Pulse Counter #, INPUT_SIG - Signal Input Pin, INPUT_CTRL - Control Input Pin,
 * Channel - Unit input channel, H_LIM - High Limit, L_LIM - Low Limit,
 * THRESH1 - configurable limit 1, THRESH0 - configurable limit 2, 
 */
void pcnt_init_channel(pcnt_unit_t PCNT_UNIT,
    int PCNT_INPUT_SIG_IO,
    int PCNT_INPUT_CTRL_IO,
    pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0,
    int PCNT_H_LIM_VAL = CNT_LIMIT,
    int PCNT_L_LIM_VAL = -CNT_LIMIT) 
    {
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config; 
        // Set PCNT input signal and control GPIOs
        pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
        pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;
        pcnt_config.channel = PCNT_CHANNEL;
        pcnt_config.unit = PCNT_UNIT;
        // What to do on the positive / negative edge of pulse input?
        pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
        pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
        pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;
    
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);

    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT);
}

void sendResponse(int x)
{
  // Send response on serial to MATLAB
  /*
    Responses to MATLAB:
    resp #  | meaning
    -1000     fatal error - stop scan
    -1        received emergency stop
    -2        start scan failed, scan in progress
    1         start scan success
    2         scan is complete (CheckScanDone in MATLAB
    3         ready for sarNextUp (CheckUpDone in MATLAB))
    4         sarNextUp received 
    5         ready for sarNextDown (CheckDownDone in MATLAB))
    6         sarNextDown received 
  */
  Serial.print("drs ");
  Serial.println(x);
}

void sendScanError()
{
  // Send fatal error code to MATLAB
  isScanning = 0;
  sendResponse(-1000);
}

int serialParse(char incomingStr[])
{
  // Parse the C-string and execute the command

  // Format: [cmd] [arg1] [arg2] ...
  // Ex:     sarStart 72 51200 0.94937 10 256 64 1 0
  //         explanation of above <start> <mm/rev> <steps/rev> <step size in mm> <offset in mm> <num x steps> <num y steps> <num radars> <separation between radars in mm>
  // Ex:     sarStop 
  // Ex:     sarNext (next horizontal scan is ready, vertical motion is done)
  
  char * command;
  char * args[8];

  // Extract the command
  command = strtok(incomingStr, " ");

  // Determine the command type and execute
  if (strcmp(command, "sarStart") == 0)
  {
    // Received sarStart, get the arguments and start the scan 
    // Get the arguments
    for (int i = 0; i < 8; i++)
    {
      args[i] = strtok(NULL, " ");
    }

    // Debug: print the arguments
    if (isDebug)
    {
      for (int i = 0; i < 8; i++)
      {
        Serial.print("drs_debug ");
        Serial.println(args[i]);
      }
    }

    // Extract the argument values
    double pulses_per_mm = atof(args[1])/atoi(args[0]);
    scan.pulses_per_rev = atoi(args[1]);
    scan.xStep_mm = atof(args[2]);
    scan.xStep_pulses = scan.xStep_mm * pulses_per_mm;
    scan.xOffset_mm = atof(args[3]);
    scan.xOffset_pulses = scan.xOffset_mm * pulses_per_mm;
    scan.numX = atoi(args[4]);
    scan.numY = atoi(args[5]);
    scan.radarSelect = atoi(args[6]);
    scan.DeltaX_mm = atof(args[7]);
    scan.DeltaX_pulses = (int)round(scan.DeltaX_mm * pulses_per_mm);

    if (scan.radarSelect == 1)
    {
      if (isDebug)
      {
        Serial.println("drs_debug Using radar 1 only");
      }

      // Using radar 1
      isRadar1 = 1;
      isRadar2 = 0;
    }
    else if (scan.radarSelect == 2)
    {
      if (isDebug)
      {
        Serial.println("drs_debug Using radar 2 only");
      }

      // Using radar 2
      isRadar1 = 0;
      isRadar2 = 1;
    }
    else if (scan.radarSelect == 3)
    {
      if (isDebug)
      {
        Serial.print("drs_debug Using dual radars with DeltaX_mm = ");
        Serial.println(scan.DeltaX_mm);
      }

      // Using both radars
      isRadar1 = 1;
      isRadar2 = 1;
    }

    // Start the puslseListenerTask
    if (!isScanning)
    {
      // Send response to MATLAB
      sendResponse(1);
      vTaskDelay(500 / portTICK_PERIOD_MS);

      xTaskCreatePinnedToCore(
        pulseListenerTask,    // function name
        "Pulse Listener",     // task name
        10000,                // stack size
        NULL,                 // task parameters
        10,                   // task priority
        &pulseListener_handle,// task handle
        1                     // task core
      );
    }
    else
    {
      // Could not start new scan, scan in progress
     sendResponse(-2);
     return -2;
    }
    return 1;
  }
  else if (strcmp(command, "sarStop") == 0)
  {
    // Received sarStop, stop the scan
    if (isDebug)
    {
      Serial.println("drs_debug received sarStop");
    }

    // Send response to MATLAB that emergency stop is received
    sendResponse(-1);
    return -1;
  }
  else if (strcmp(command, "sarNextUp") == 0)
  {
    if (isDebug)
    {
      Serial.println("drs_debug received sarNextUp");
    }

    // Send response to MATLAB
    sendResponse(4);
    return 4;
  }
  else if (strcmp(command, "sarNextDown") == 0)
  {
    if (isDebug)
    {
      Serial.println("drs_debug received sarNextDown");
    }

    // Send response to MATLAB
    sendResponse(6);
    return 6;
  }
  else
  {
    // Received unknown / illegal command, go back to listening to serial
    return 0;
  }
}

void sendTriggerTask1(void * parameters)
{
  // Send trigger to radar 1
  // This is the high priority task

  // Send the trigger
  GPIO.out_w1ts = ((uint32_t)1 << RADAR1_TRIGGER_PIN);
  GPIO.out_w1tc = ((uint32_t)1 << RADAR1_TRIGGER_PIN);

  // End task
  vTaskDelete(NULL);
}

void sendTriggerTask2(void * parameters)
{
  // Send Trigger to radar 2
  // This is the high priority task

  // Send the trigger
  GPIO.out_w1ts = ((uint32_t)1 << RADAR2_TRIGGER_PIN);
  GPIO.out_w1tc = ((uint32_t)1 << RADAR2_TRIGGER_PIN);

  // End task
  vTaskDelete(NULL);
}

int getNextBreakpoint(int counterTriggers, int counterPulses, int dir)
{
  int temp = (int)round(counterTriggers * scan.xStep_pulses + scan.xOffset_pulses + X_ALWAYS_OFFSET);

  if (isDebug)
  {
    Serial.print("drs_debug nextBreakpoint = ");
    Serial.println(temp);
  }
  
  if (dir == 1 && counterPulses <= temp)
  {
    // Counting up and have not exceeded the next breakpoint
    return temp;
  }
  else if (dir == -1 && counterPulses >= temp)
  {
    // Counting down and have not passed the next breakpoint
    return temp;
  }
  else
  {
    // Passed the next breakpoint without sending trigger - BAD
    // Send fatal error to MATLAB
    sendScanError();

    // Delete this task
    vTaskDelete(NULL);
    return 0;
  }
  return 0;
}

void printErrorDebug(int16_t count, int counterPulses, int multPulses, int nextBreakpoint1, int nextBreakpoint2)
{
  if (isDebugError)
  {
    Serial.print("count = ");
    Serial.println(count);
    Serial.print("counterPulses = ");
    Serial.println(counterPulses);
    Serial.print("multPulses = ");
    Serial.println(multPulses);
    Serial.print("nextBreakpoint1 = ");
    Serial.println(nextBreakpoint1);
    Serial.print("nextBreakpoint2 = ");
    Serial.println(nextBreakpoint2);
  }
}

void pulseListenerTask(void * parameters) 
{
  // This is a high priority task

  /*
  // Reset the counter to 0
  pcnt_counter_pause(PCNT_TEST_UNIT);
  pcnt_counter_clear(PCNT_TEST_UNIT);
  pcnt_intr_enable(PCNT_TEST_UNIT);

  // Everything is set up, now go to counting
  pcnt_counter_resume(PCNT_TEST_UNIT);*/

  pcnt_counter_clear(PCNT_TEST_UNIT);

  // isScanning = true
  isScanning = 1;

  // Reset isNext
  isNext = 0;

  // Counter, limited by CNT_LIMIT
  int16_t count = 0;
  int16_t countOld = -1;

  // Pulse counter - basically unlimited, uses overflow protection
  int counterPulses = 0;
  int multPulses = 0;
  int dir = 1;
  int nextBreakpoint1 = 0;
  int nextBreakpoint2 = 0;
  bool isUpComplete1 = 0;
  bool isDownComplete1 = 0;
  bool isUpComplete2 = 0;
  bool isDownComplete2 = 0;
  bool isWait1 = 0;
  bool isWait2 = 0;

  // Command from MATALB during scan
  int cmd = 0;

  // Trigger counter
  long counterTriggers1 = 0;
  long counterTriggers2 = 0;

  // Lap counter (1 lap = 2 y steps)
  long counterLaps = 0;

  nextBreakpoint1 = getNextBreakpoint(counterTriggers1, counterPulses, dir);
  nextBreakpoint2 = getNextBreakpoint(counterTriggers2, (counterPulses - scan.DeltaX_pulses), dir);

  pcnt_get_counter_value(PCNT_TEST_UNIT, &count);

  for(;;)
  {
    /* Wait for the event information passed from PCNT's interrupt handler.
    * Once received, decode the event type and print it on the serial monitor.
    * While waiting, continuously print the counter
    */
    
    // Assign the current counter value to count
    pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
    
    // Use overflow protection
    res = xQueueReceive(pcnt_evt_queue, &evt, 0 / portTICK_PERIOD_MS);
    if (res == pdTRUE) {
      if (evt.status & PCNT_STATUS_H_LIM_M) 
      {
        multPulses++;
      }
      if ((evt.status & PCNT_EVT_H_LIM) && countOld < -CNT_LIMIT/8) // Makes no sense why we use this keyword, but it works
      {
        multPulses--;
      }
      pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
    }

    // Update pulse counter
    counterPulses = multPulses * CNT_LIMIT + count;

    // Check if a serial command has been sent
    if (Serial.available() > 0)
    {
      // Read the incoming line
      incomingMessage = Serial.readString();
      incomingMessage.replace("\n", "");
      incomingMessage.toCharArray(incomingStr, sizeof(incomingStr));

      // Parse the input and execute the command
      cmd = serialParse(incomingStr);

      // Do command
      if (cmd == -1)
      {
        // Stop scan
        isScanning = 0;

        // Delete this task
        vTaskDelete(NULL);
      }
      else if (cmd == 1)
      {
        // Send fatal error to MATLAB
        sendScanError(); 
        if (isDebugError)
        {
          Serial.println("drs_debug Attempted to start scan during another scan!"); 
        }

        printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);

        // Delete this task
        vTaskDelete(NULL);  
      }
      else if (cmd == 4)
      {
        // sarNextUp received - stop radars waiting
        isWait1 = 0;
        isWait2 = 0;

        dir = -1;
        printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);
      }
      else if (cmd == 6)
      {
        // sarNextDown received - stop radars waiting
        isWait1 = 0;
        isWait2 = 0;

        dir = 1;
        printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);
      }
    }

    // Debug - print output
    if (isDebug && count != countOld)
    {
      Serial.print("drs_debug radar 1: counterPulses = ");
      Serial.println(counterPulses);
      Serial.print("drs_debug radar 2: (counterPulses - scan.DeltaX_pulses) = ");
      Serial.println(counterPulses - scan.DeltaX_pulses);
    }
    
    // Set countOld
    countOld = count;

    // Exit if counterPulses is negative somehow!
    if (counterPulses < 0)
    {
      // Send fatal error to MATLAB
      sendScanError(); 
      if (isDebugError)
      {
        Serial.println("drs_debug counterPulses is negative somehow"); 
      }

      printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);

      // Delete this task
      vTaskDelete(NULL);
    }

    // Check if we are at the next breakpoint for radar 1
    if (isRadar1 && counterPulses == nextBreakpoint1 && !isWait1) 
    {
      // Send the trigger
      xTaskCreate(
        sendTriggerTask1,     // function name
        "Trigger Radar 1",    // task name
        1000,                 // stack size
        NULL,                 // task parameters
        10,                   // task priority
        &sendTrigger1_handle  // task handle
      );
      
      // Update the trigger counter
      if (dir == 1)
      {
        // Debug
        if (isDebug)
        {
          Serial.print("drs_debug Radar 1 upward trigger #");
          Serial.println(counterTriggers1);
          printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);
        }

        // Check if all triggers have been sent
        if (counterTriggers1 == (scan.numX-1))
        {
          // Completed the correct number of triggers
          isUpComplete1 = 1;

          // Start waiting for next
          if (isDebugWaiting)
          {
            Serial.println("drs_debug radar 1 is waiting for sarNextUp");
          }
          // Wait for the sarNextUp command from MATLAB 
          if (!isRadar2)
          {
            sendResponse(3);
          }
          isWait1 = 1;
        }
        else if (counterTriggers1 < (scan.numX-1))
        {
          // If we are not on the last trigger, increment the trigger counter
          counterTriggers1++;
        }
        else if (counterTriggers1 > (scan.numX-1))
        {
          // Completed too many triggers - Throw an error
          // Send fatal error to MATLAB
          sendScanError();
          if (isDebugError)
          {
            Serial.println("drs_debug Radar 1 completed too many triggers");
          }           

          // Delete this task
          vTaskDelete(NULL);
        }
      }
      else if (dir == -1)
      {
        // Debug
        if (isDebug)
        {
          Serial.print("drs_debug Radar 1 downward trigger #");
          Serial.println(counterTriggers1);
          printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);
        }

        if (!isUpComplete1 || (isRadar2 && !isUpComplete2))
        {
          // Did not trigger the correct number of upward triggers! BAD
          // Send fatal error to MATLAB
          sendScanError(); 
          if (isDebugError) 
          {
            Serial.println("drs_debug Radar 1 did not trigger the correct number of upward triggers");
          }          

          // Delete this task
          vTaskDelete(NULL);
        }

        if (counterTriggers1 == 0){
          // Check if we are using radar 2 and it did not complete its downward scan! BAD
          if (isRadar2 && !isDownComplete2)
          {
            // Send fatal error to MATLAB
            sendScanError(); 
            if (isDebugError) 
            {
              Serial.println("drs_debug Radar 2 did not complete enough downward triggers before radar 1 finished!");
            }          

            // Delete this task
            vTaskDelete(NULL);
          }
          else 
          {
            // Completed the correct number of triggers
            isDownComplete1 = 1;

            // Start waiting for next
            if (isDebugWaiting)
            {
              Serial.println("drs_debug radar 1 is waiting for sarNextDown");
            }
            // Wait for the sarNextDown command from MATLAB 
            sendResponse(5);
            isWait1 = 1;  
          }
        }
        else if (counterTriggers1 > 0)
        {
          // If we are not on the last downward trigger, decrement the trigger counter
          counterTriggers1--;
        }
        else if (counterTriggers1 < 0)
        {
          // Completed too many triggers - Throw an error
          // Send fatal error to MATLAB
          sendScanError(); 
          if (isDebugError)
          {
            Serial.println("drs_debug Radar 1 completed too many downward triggers");
          }

          // Delete this task
          vTaskDelete(NULL);
        }
      }

      // Check if we have gone up and then down (one complete down and back scan)
      if (isUpComplete1 && isDownComplete1)
      {
        // Check if we are using radar 2 and it did not complete both up and down scans
        if (isRadar2 && !(isUpComplete2 && isDownComplete2))
        {
          // Send fatal error to MATLAB
          sendScanError(); 
          if (isDebugError)
          {
            Serial.println("drs_debug Radar 2 did not complete lap and radar 1 did");
          }

          // Delete this task
          vTaskDelete(NULL);
        }
        // Else if we are using radar 1 and it did complete up and down scans
        else if (isRadar2 && (isUpComplete2 && isDownComplete2))
        {
          // Reset for next
          isUpComplete2 = 0;
          isDownComplete2 = 0;
        }
        // Reset for next 
        isUpComplete1 = 0;
        isDownComplete1 = 0;

        // Increment lap counter
        counterLaps++;
        if (isDebug)
        {
          Serial.print("counterLaps = ");
          Serial.println(counterLaps);
        }

        // Check if lap counter has met the expected value
        if (2*counterLaps >= scan.numY) 
        {
          // Scan is complete - exit the scan
          isScanning = 0;

          // Send scan complete response MATLAB
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          sendResponse(2);

          // Break from while loop
          break;
        }
      }
      
      nextBreakpoint1 = getNextBreakpoint(counterTriggers1, counterPulses, dir);
    }
    else if (isRadar1 && (counterPulses > nextBreakpoint1) && dir == 1 && !isWait1)
    {
      // We passed the next breakpoint counting up
      // Send fatal error to MATLAB
      sendScanError(); 
      if (isDebugError)
      {
        Serial.println("drs_debug Radar 1 passed the next breakpoint counting up");
      }      

      printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);

      // Delete this task
      vTaskDelete(NULL);
    }
    else if (isRadar1 && (counterPulses < nextBreakpoint1) && dir == -1 && !isWait1)
    {
      // We passed the next breakpoint counting down
      // Send fatal error to MATLAB
      sendScanError(); 
      if (isDebugError)
      {
        Serial.println("drs_debug Radar 1 passed the next breakpoint counting down");
      }      

      // Delete this task
      vTaskDelete(NULL);
    }
  
    // Check if we are at the next breakpoint for radar 2
    if (isRadar2 && (counterPulses - scan.DeltaX_pulses) == nextBreakpoint2 && !isWait2) 
    {
      // Send the trigger
      xTaskCreate(
        sendTriggerTask2,     // function name
        "Trigger Radar 2",    // task name
        1000,                 // stack size
        NULL,                 // task parameters
        10,                   // task priority
        &sendTrigger2_handle  // task handle
      );
      
      // Update the trigger counter
      if (dir == 1)
      {
        // Debug
        if (isDebug)
        {
          Serial.print("drs_debug Radar 2 upward trigger #");
          Serial.println(counterTriggers2);
          printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);
        }

        // Check if all triggers have been sent
        if (counterTriggers2 == (scan.numX-1))
        {
          // Check if we are using radar 1 and it did not complete its upward scan! BAD
          if (isRadar1 && !isUpComplete1)
          {
            // Send fatal error to MATLAB
            sendScanError(); 
            if (isDebugError) 
            {
              Serial.println("drs_debug Radar 1 did not complete enough upward triggers before radar 2 finished!");
            }          

            // Delete this task
            vTaskDelete(NULL);
          }
          else
          {
            // Completed the correct number of triggers
            isUpComplete2 = 1;

            // Start waiting for next
            if (isDebugWaiting)
            {
              Serial.println("drs_debug radar 2 is waiting for sarNextUp");
            }
            // Wait for the sarNextUp command from MATLAB 
            sendResponse(3);
            isWait2 = 1;
          }
        }
        else if (counterTriggers2 < (scan.numX-1))
        {
          // If we are not on the last trigger, increment the trigger counter
          counterTriggers2++;
        }
        else if (counterTriggers2 > (scan.numX-1))
        {
          // Completed too many triggers - Throw an error
          // Send fatal error to MATLAB
          sendScanError();
          if (isDebugError)
          {
            Serial.println("drs_debug Radar 2 completed too many triggers");
          }           

          // Delete this task
          vTaskDelete(NULL);
        }
      }
      else if (dir == -1)
      {
        // Debug
        if (isDebug)
        {
          Serial.print("drs_debug Radar 2 downward trigger #");
          Serial.println(counterTriggers2);
          printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);
        }

        if (!isUpComplete2 || (isRadar1 && !isUpComplete1))
        {
          // Did not trigger the correct number of upward triggers! BAD
          // Send fatal error to MATLAB
          sendScanError(); 
          if (isDebugError) 
          {
            Serial.println("drs_debug Radar 2 did not trigger the correct number of upward triggers");
          }          

          // Delete this task
          vTaskDelete(NULL);
        }

        if (counterTriggers2 == 0){
          // Completed the correct number of triggers
          isDownComplete2 = 1;

          // Start waiting for next
          if (isDebugWaiting)
          {
            Serial.println("drs_debug radar 2 is waiting for sarNextDown");
          }
          // Wait for the sarNextDown command from MATLAB 
          if (!isRadar1)
          {
            sendResponse(5);
          }          
          isWait2 = 1;
        }
        else if (counterTriggers2 > 0)
        {
          // If we are not on the last downward trigger, decrement the trigger counter
          counterTriggers2--;
        }
        else if (counterTriggers2 < 0)
        {
          // Completed too many triggers - Throw an error
          // Send fatal error to MATLAB
          sendScanError(); 
          if (isDebugError)
          {
            Serial.println("drs_debug Radar 2 completed too many triggers");
          }

          // Delete this task
          vTaskDelete(NULL);
        }
      }

      // Check if we have gone up and then down (one complete down and back scan)
      if (!isRadar1 && (isUpComplete2 && isDownComplete2))
      {
        // Reset for next 
        isUpComplete2 = 0;
        isDownComplete2 = 0;

        // Increment lap counter
        counterLaps++;
        if (isDebug)
        {
          Serial.print("counterLaps = ");
          Serial.println(counterLaps);
        }

        // Check if lap counter has met the expected value
        if (2*counterLaps >= scan.numY) 
        {
          // Scan is complete - exit the scan
          isScanning = 0;

          // Send scan complete response MATLAB
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          sendResponse(2);

          // Break from while loop
          break;
        }
      }
      
      nextBreakpoint2 = getNextBreakpoint(counterTriggers2, (counterPulses - scan.DeltaX_pulses), dir);
    }
    else if (isRadar2 && ((counterPulses - scan.DeltaX_pulses) > nextBreakpoint2) && dir == 1 && !isWait2)
    {
      Serial.print("count = ");
      Serial.println(count);

      // We passed the next breakpoint counting up
      // Send fatal error to MATLAB
      sendScanError(); 
      if (isDebugError)
      {
        Serial.println("drs_debug Radar 2 passed the next breakpoint counting up");
      }      

      printErrorDebug(count, counterPulses, multPulses, nextBreakpoint1, nextBreakpoint2);

      // Delete this task
      vTaskDelete(NULL);
    }
    else if (isRadar2 && ((counterPulses - scan.DeltaX_pulses) < nextBreakpoint2) && dir == -1 && !isWait2)
    {
      // We passed the next breakpoint counting down
      // Send fatal error to MATLAB
      sendScanError(); 
      if (isDebugError)
      {
        Serial.println("drs_debug Radar 2 passed the next breakpoint counting down");
      }      

      // Delete this task
      vTaskDelete(NULL);
    }
  }
  
  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);

  pinMode(RADAR1_TRIGGER_PIN, OUTPUT);
  pinMode(RADAR2_TRIGGER_PIN, OUTPUT);

  /* Initialize PCNT event queue and PCNT functions */
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
  pcnt_init_channel(PCNT_TEST_UNIT, STEPPER_PIN, CTRL_PIN); // Initialize Unit 0 to STEPPER_PIN
}

void loop()
{
  // Check if a serial command has been sent
  if (Serial.available() > 0)
  {
    // Read the incoming line
    incomingMessage = Serial.readString();
    incomingMessage.replace("\n", "");
    incomingMessage.toCharArray(incomingStr, sizeof(incomingStr));

    // Parse the input and execute the command
    serialParse(incomingStr);
  }

  // Wait one second between 
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  yield();
}


