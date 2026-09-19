/* ==================================================================== */
/* ============================= INCLUDES ============================= */
/* ==================================================================== */
#include "main.h"
#include "j1772.h"
#include "GopherCAN.h"

/* ==================================================================== */
/* ============================= DEFINES ============================== */
/* ==================================================================== */

// J1772 charging Control Pilot (CP) signal standard values
// J1772 generates a 1 kHz full cycle CP PWM signal

// Timeout of CP PWM signal in milliseconds
#define CP_PWM_TIMOUT_MICROSEC   (10U)

// Maximum continuous ampere rating is 0.6 A per 10 us of high time for PWM signal 
#define CP_CURRENT_PER_MICROSEC   (0.06f)  // 0.6 A per 10 us = 0.06 A per 1 us

// Minimum and maximum allowed high time for formula
#define CP_MIN_HIGH_TIME_MICROSEC    (100U)
#define CP_MAX_HIGH_TIME_MICROSEC    (850U)

// Standard voltage for J1772 AC level 1 and 2 charging
#define STANDARD_LVL1_VOLT (120.0f)
#define STANDARD_LVL2_VOLT (208.0f)

// Maximum current for J1772 AC level 1 charging
#define STANDARD_LVL1_AMP (16.0f)

// Standard NEMA maximum current in Amps and voltage in Volts
// The BMS will assume the standard power limit for NEMA unless it receives a message
// from the charger board indicating the possibility of a higher power limit
#define DEFAULT_RATED_CURRENT_AMP   (15.0f)
// Limit max current to 80% of rated current
#define DEFAULT_CONT_CURRENT_AMP (12.0f)
#define DEFAULT_VOLTAGE_VOLT (110.0f)

// Absolute maximum current of 20 Amps to comply with FSAE comp rules (20A breaker)
#define MAX_RATED_CURRENT_AMP (20.0f)
// Limit max current further
// FSAE student manual says typical continuous current is about 80% of rated current
#define MAX_CONT_CURRENT_AMP (16.0f)


/* ==================================================================== */
/* ======================= EXTERNAL VARIABLES ========================= */
/* ==================================================================== */

// Voltatile variables to be updated from IC timer interrupt
extern volatile uint32_t cpFrequency;
extern volatile uint32_t cpHighTime;
extern volatile uint32_t cpDutyCycle;
extern volatile uint32_t cpLastUpdate;

extern CAN_HandleTypeDef hcan2;

/* ==================================================================== */
/* =================== GLOBAL FUNCTION DEFINITIONS ==================== */
/* ==================================================================== */

/*!
  @brief    Determine the maximum power limit for charging
  @param    chargingData Data struct to return charging info
*/

void getJ1772Status(chargingData_S *chargingData)
{
        // Manually set power limit
        chargingData->powerLimit = 6000.0f;

    if((HAL_GetTick() - cpLastUpdate) < CP_PWM_TIMOUT_MICROSEC && cpLastUpdate != 0)
	{
		// HAL GPIO -> ENABLE CP
        HAL_GPIO_WritePin(CP_EN_GPIO_Port, CP_EN_Pin, GPIO_PIN_SET);

    //     // Copy local of volatile variable
    //     uint32_t localHighTime = cpHighTime;

    //     // Calculate current limit
    //     if (localHighTime < CP_MIN_HIGH_TIME_MICROSEC)
    //     {
    //         localHighTime = CP_MIN_HIGH_TIME_MICROSEC;
    //     }
    //     else if (localHighTime > CP_MAX_HIGH_TIME_MICROSEC)
    //     {
    //         localHighTime = CP_MAX_HIGH_TIME_MICROSEC;
    //     }

    //     // Calculate max current capacity
    //     chargingData->maxAmpacity = cpHighTime * CP_CURRENT_PER_MICROSEC;

    //     // Clamp max current to avoid tripping 20A breaker at comp
    //     if(chargingData->maxAmpacity > MAX_CONT_CURRENT_AMP)
    //     {
    //         chargingData->maxAmpacity = MAX_CONT_CURRENT_AMP;
    //     }

    //     // Calculate power limit
    //     // At comp the J1772 EVSE will be 208V
    //     chargingData->powerLimit = STANDARD_LVL2_VOLT * chargingData->maxAmpacity;

    //     // if(chargingData->maxAmpacity <= STANDARD_LVL1_AMP)
    //     // {
    //     //     // Level 1 charging
    //     //     chargingData->powerLimit = STANDARD_LVL1_VOLT * chargingData->maxAmpacity;
    //     // }
    //     // else
    //     // {
    //     //     // Level 2 charging
    //     //     chargingData->powerLimit = STANDARD_LVL2_VOLT * chargingData->maxAmpacity;
    //     // }
	
    } else {
        // No J1772 detected, charge at default NEMA values

        // HAL GPIO -> DISABLE CP
        HAL_GPIO_WritePin(CP_EN_GPIO_Port, CP_EN_Pin, GPIO_PIN_RESET);

        // // Set power limit to default
        // chargingData->maxAmpacity = DEFAULT_CONT_CURRENT_AMP;
        // chargingData->powerLimit = DEFAULT_VOLTAGE_VOLT * DEFAULT_CONT_CURRENT_AMP;

        // // Or manually set power limit
        // chargingData->powerLimit = 1000.0f;
    }

    // Send charging power limit to BMS over CAN
    // Note: was not sending so fixed by calling update_and_queue in serviceGcanTask
    // Note: see serviceGcanTask for second can debug fix
    // update_and_queue_param_float(&chargingPowerLimit,chargingData->powerLimit);

}

