#include <stdint.h>
#include "main.h"
#include "idleTask.h"
#include <stdio.h>
#include "GopherCAN.h"

#define HEARTBEAT_BLINK_MS      300
#define HEARTBEAT_PERIOD_MS     2000

void initIdleTask(){
    HAL_GPIO_WritePin(Heartbeat_GPIO_Port, Heartbeat_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET); // green led normally low

}
void runIdleTask(){
    static uint8_t hbState = 0;
    static  uint32_t lastHeartBeatUpdate = 0;
    if(hbState){
        if(HAL_GetTick() - lastHeartBeatUpdate > HEARTBEAT_BLINK_MS){
            lastHeartBeatUpdate = HAL_GetTick();
            HAL_GPIO_WritePin(Heartbeat_GPIO_Port, Heartbeat_Pin, GPIO_PIN_RESET);
            hbState = 0;
        }

    } else {
        if(HAL_GetTick() - lastHeartBeatUpdate > (HEARTBEAT_PERIOD_MS - HEARTBEAT_BLINK_MS)){
            lastHeartBeatUpdate = HAL_GetTick();            
            HAL_GPIO_WritePin(Heartbeat_GPIO_Port, Heartbeat_Pin, GPIO_PIN_SET);
            hbState = 1;
        }
    }

   

    // For charging
    // printf("\e[1;1H\e[2J");
    // printf("soe by ocv : %f\n", soeByOCV_percent.data * 100.0f);
    // printf("soe by cc : %f\n", soeByCoulombCounting_percent.data * 100.0f);
    // printf("pack voltage: %f\n", bmsBatteryVoltage_V.data);
    // printf("tractive system voltage: %f\n", bmsTractiveSystemVoltage_V.data);
    // printf("pack current: %f\n", batteryCurrent_A.data);
    
    // printf("hello world\n");
    // printf("soe: %f\n", 3.14f);

    // For debugging sdcStatus
    // printf("\e[1;1H\e[2J");
    // printf("SDC Status:\n");
    // printf("1: %u\n2: %u\n3: %u\n4: %u\n", sdcStatus17.data, sdcStatus18.data, sdcStatus19.data);   
    // osDelay(500);

    // For debugging J1772
    static uint32_t lastUpdate = 0;
    if((HAL_GetTick() - lastUpdate) > 500){
        // printf("\e[1;1H\e[2J");
        // printf("Max Ampactiy: %f\n", chargingData.maxAmpacity);
        // printf("Charging Power Limit: %f\n", chargingPowerLimit.data);
        // printf("\e[1;1H\e[2J");
        // printf("soe by ocv : %f\n", soeByOCV_percent.data);
        // printf("soe by cc : %f\n", soeByCoulombCounting_percent.data);
        // printf("pack voltage: %f\n", bmsBatteryVoltage_V.data);
        // printf("tractive system voltage: %f\n", bmsTractiveSystemVoltage_V.data);
        // printf("pack current: %f\n", bmsBatteryCurrent_A.data);
        // printf("max cell voltage 1: %f\n", segment1MaxCellVoltage_V.data);
        // printf("max cell voltage 2: %f\n", segment2MaxCellVoltage_V.data);
        // printf("max cell voltage 3: %f\n", segment3MaxCellVoltage_V.data);
        // printf("max cell voltage 4: %f\n", segment4MaxCellVoltage_V.data);
        // printf("max cell voltage 5: %f\n", segment5MaxCellVoltage_V.data);
        // printf("max cell voltage 6: %f\n", segment6MaxCellVoltage_V.data);
        // printf("max cell voltage 7: %f\n", segment7MaxCellVoltage_V.data);
        // printf("max cell voltage 8: %f\n", segment8MaxCellVoltage_V.data);
        // printf("max cell temp 1: %f\n", segment1MaxCellTemperature_C.data);
        // printf("max cell temp 2: %f\n", segment2MaxCellTemperature_C.data);
        // printf("max cell temp 3: %f\n", segment3MaxCellTemperature_C.data);
        // printf("max cell temp 4: %f\n", segment4MaxCellTemperature_C.data);
        // printf("max cell temp 5: %f\n", segment5MaxCellTemperature_C.data);
        // printf("max cell temp 6: %f\n", segment6MaxCellTemperature_C.data);
        // printf("max cell temp 7: %f\n", segment7MaxCellTemperature_C.data);
        // printf("max cell temp 8: %f\n", segment8MaxCellTemperature_C.data);
        // printf("chargingPowerLimit: %f\n", chargingPowerLimit.data);
        // printf("last rx %lu", chargingPowerLimit.info.last_tx);

        printf("\e[1;1H\e[2J");

        printf("soe OCV: %f\n", soeByOCV_percent.data);
        printf("soe CC: %f\n", soeByCoulombCounting_percent.data);
        printf("bms battery current: %f\n\n", bmsBatteryCurrent_A.data);

        printf("Max Cell V: %f\n", maxCellVoltage_V.data);
        printf("Min Cell V: %f\n", minCellVoltage_V.data);
        printf("Avg Cell V: %f\n", avgCellVoltage_V.data);
        printf("Cell Imbalance mV: %f\n\n", cellImbalance_mV.data);

        printf("Max Cell Temp: %f\n", maxCellTemp_C.data);
        printf("Min Cell Temp: %f\n", minCellTemp_C.data);
        printf("Avg Cell Temp: %f\n\n", avgCellTemp_C.data);

        printf("Max Board Temp: %f\n", maxBoardTemp_C.data);
        printf("Min Board Temp: %f\n", minBoardTemp_C.data);
        printf("Avg Board Temp: %f\n\n", avgBoardTemp_C.data);

        printf("BMB0 Die Temp: %f\n", segment1DieTemperature_C.data);
        printf("BMB1 Die Temp: %f\n", segment2DieTemperature_C.data);
        printf("BMB2 Die Temp: %f\n", segment3DieTemperature_C.data);
        printf("BMB3 Die Temp: %f\n", segment4DieTemperature_C.data);
        printf("BMB4 Die Temp: %f\n", segment5DieTemperature_C.data);
        printf("BMB5 Die Temp: %f\n", segment6DieTemperature_C.data);
        printf("BMB6 Die Temp: %f\n", segment7DieTemperature_C.data);
        printf("BMB7 Die Temp: %f\n", segment8DieTemperature_C.data);

        // osDelay(500);
        lastUpdate = HAL_GetTick();
    }

    
}
