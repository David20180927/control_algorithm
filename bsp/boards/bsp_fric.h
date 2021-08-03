#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

//Fric speed, 1000~2000? 
#define FRIC_UP 1500 //original 1400
#define FRIC_DOWN 1470 //original 1320
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
