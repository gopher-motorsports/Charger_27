
#include <stdint.h>
#include "main.h"
#include "sdc.h"
#include <stdio.h>
#include "GopherCAN.h"

static uint8_t statSDC17 = 0;
static uint8_t statSDC18 = 0;
static uint8_t statSDC19 = 0;

void initSdc(){

}
void runSdc(){
    // check status of each SDC pin
    // if the pin is low, SDC status is good -> do nothing
    // if the pin is high, SDC status is bad -> act accordingly
    statSDC17 = HAL_GPIO_ReadPin(SDC1_GPIO_Port,SDC1_Pin);
    statSDC18 = HAL_GPIO_ReadPin(SDC2_GPIO_Port,SDC2_Pin);
    statSDC19 = HAL_GPIO_ReadPin(SDC3_GPIO_Port,SDC3_Pin);

    update_and_queue_param_u8(&sdcStatus17,statSDC17);
    update_and_queue_param_u8(&sdcStatus18,statSDC18);
    update_and_queue_param_u8(&sdcStatus19,statSDC19);


}
