#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stdio.h"
#include "string.h"

RCC_ClocksTypeDef RCC_Clocks;
int ms_Ticks; /* counts 1ms timeTicks       */
   Responses blue tooth variables
   char BT_CMD  = 1;
   char BT_AOK  = 2;
   char BT_CONN = 3;
   char BT_Done = 4;
   char BT_state = 0;
   
   char response_rcvd = 0;
