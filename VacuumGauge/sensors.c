
#include "sensors.h"
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

struct sens{
	float value;
	float(*func)(int id);
	char str[16];		
};

static sensor_s *sensors = NULL;
static int sensors_count = 0;
static float unit_coeff[3] = {1.0f, 0.01f, 0.007501f};
static int filter_coeff = 3; //коэффициент фильтра скользящего среднего

void addSensors(int num)
{
	sensors = realloc(sensors, (num + sensors_count)*sizeof(sensor_s));
	for (int i = 0; i < num; i++){
		sensors[sensors_count+i].func = NULL;
		sensors[sensors_count+i].value = -1.0f;
	}
	sensors_count += num;
}

float sensor(int id, pressure_unit unit)
{
	if (id < sensors_count){
		return unit_coeff[unit]*sensors[id].value;
	}
	else{
		return -1;
	}
}

void sensorCalc(int id)
{
	if ((id < sensors_count) && (sensors[id].func != NULL)){
		float new_value = sensors[id].func(id);
		if (new_value > 0)
			if (sensors[id].value < 0)
				sensors[id].value = new_value;
			else
				sensors[id].value = ((float)(filter_coeff - 1)*sensors[id].value + new_value)/(float)filter_coeff;
		else{
			sensors[id].value = -1;
		}
	}
}

char* sensor_str(int id, pressure_unit unit)
{
	if (id < sensors_count){
		float temp_value = unit_coeff[unit]*sensors[id].value;
		char buf[16] = {0};
		sprintf(sensors[id].str, "P%d = ", id + 1);
		if (temp_value < 0){
			strcat(sensors[id].str, "---");
		}
		else{
			int power = 0;
			if ((temp_value >= 1) && (temp_value <= 9999)){
				dtostrf(temp_value, 1, temp_value>=10?0:2, buf);
				strcat(sensors[id].str, buf);
			}
			else if (temp_value < 1){
				for (;(temp_value < 1) && (power < 5); temp_value *= 10, power++){}
				dtostrf(temp_value, 1, 2, buf);
				char symb_power[] = {36, 160 + power, '\0'};
				strcat(sensors[id].str, buf);
				strcat(sensors[id].str, "x10");
				strcat(sensors[id].str, symb_power);
			}
			else if (temp_value > 9999){
				for (;(temp_value >= 100) && (power < 4); temp_value /= 10, power++){}
				dtostrf(temp_value, 1, 1, buf);
				char symb_power[] = {160 + power, '\0'};
				strcat(sensors[id].str, buf);
				strcat(sensors[id].str, "x10");
				strcat(sensors[id].str, symb_power);
			}
		}
		return sensors[id].str;
    }
    else{
	    return NULL;
    }
}

void setCalcFunc(int id, float(*func)(int id))
{
	if (id < sensors_count){
		sensors[id].func = func;
	}
}

void deleteSensors()
{
	free(sensors);
}
