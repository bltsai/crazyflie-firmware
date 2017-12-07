#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "led_indicator.h"
#include "system.h"
#include "param.h"
#include "log.h"

#include "stm32fxxx.h"

#define LEDICTRNUM 3

static bool isInit = false;
//static bool isBlink = false;
static bool ledStatus[3] = {false, false, false};
//static bool ledIctr[3] = {false, false, false};
static uint8_t ledGPIONum[3] = {DECK_GPIO_IO1, DECK_GPIO_IO2, DECK_GPIO_IO4};
static uint8_t ledctrl = 0;

static void ledIndicatorTask(void* param)
{
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_LED_INDICATOR_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();

  while(1)
  {
    for (int i = 0; i < 3; i++) {
      if (ledctrl & (1<<(i*2))) { //led on
        if (ledctrl & (1<<(i*2+1))) { //led blink
          if (ledStatus[i]) {
            digitalWrite(ledGPIONum[i], LOW);
          } else {
            digitalWrite(ledGPIONum[i], HIGH);
          }
          ledStatus[i] = !ledStatus[i];
        } else {
          digitalWrite(ledGPIONum[i], HIGH);
        }
      } else {
        digitalWrite(ledGPIONum[i], LOW);
      }
    }
    /*
    if (!isBlink) {
      for(int i=0; i<LEDICTRNUM; i++){
        if(ledIctr[i]){
          digitalWrite(ledGPIONum[i], HIGH);
        }else{
          digitalWrite(ledGPIONum[i], LOW);
        }
      }
    } else {
      for(int i=0; i<LEDICTRNUM; i++){
        if(ledIctr[i]){
          if (ledStatus[i]) {
            digitalWrite(ledGPIONum[i], LOW);
          } else {
            digitalWrite(ledGPIONum[i], HIGH);
          }
          ledStatus[i] = !ledStatus[i];
        }else{
          digitalWrite(ledGPIONum[i], LOW);
        }
      }
    }*/
    vTaskDelayUntil(&lastWakeTime, F2T(LED_INDICATOR_TASK_FREQ));
  }
}

void ledIndicatorInit(void)
{
  if(isInit)
    return;

  for(int i=0; i<LEDICTRNUM; i++){
    pinMode(ledGPIONum[i], OUTPUT);
  }

  /* Only start the task if the proximity subsystem is enabled in conf.h */
  xTaskCreate(ledIndicatorTask, LEDINDICATOR_TASK_NAME,
              LEDINDICATOR_TASK_STACKSIZE, NULL, LEDINDICATOR_TASK_PRI, NULL);

  isInit = true;
}

PARAM_GROUP_START(ledIctr)
PARAM_ADD(PARAM_UINT8, ledctrl, &ledctrl)
//PARAM_ADD(PARAM_UINT8, ledIctrRed, &ledIctr[0])
//PARAM_ADD(PARAM_UINT8, ledIctrYellow, &ledIctr[1])
//PARAM_ADD(PARAM_UINT8, ledIctrGreen, &ledIctr[2])
PARAM_GROUP_STOP(ledIctr)
