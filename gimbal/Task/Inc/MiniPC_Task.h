#ifndef MiniPC_TASK_H
#define MiniPC_TASK_H

#include "stdint.h"
#include "stdbool.h"

typedef struct
{
	uint8_t Zone_Status; 
uint8_t Area_Status; 
uint8_t Mode_Defense;
	
	
float Hp_test;
float Hp_test_1;

float Hp_test_3;

float Hp_test_7;

uint8_t Start;

}G_Typedef;

/* Exported variables ----------------------------------------------------------*/
extern G_Typedef G;






#endif