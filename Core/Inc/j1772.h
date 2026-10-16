#ifndef INC_J1772_H_
#define INC_J1772_H_

/* ==================================================================== */
/* ============================= INCLUDES ============================= */
/* ==================================================================== */

#include <stdint.h>
#include <stdbool.h>

/* ==================================================================== */
/* ============================== STRUCTS============================== */
/* ==================================================================== */

typedef struct
{
    float maxAmpacity; // in Amps
    float powerLimit; // in Watts
} chargingData_S;

/* ==================================================================== */
/* =================== GLOBAL FUNCTION DECLARATIONS =================== */
/* ==================================================================== */

/*!
  @brief    Determine the maximum allowed current
  @param    chargingData Data struct to return J1772 and other charging info
*/

void getJ1772Status(chargingData_S *chargingData);

#endif /* INC_J1772_H_ */