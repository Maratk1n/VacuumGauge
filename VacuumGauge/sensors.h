
#ifndef SENSORS_H_
#define SENSORS_H_


typedef enum {Pa, mbar, Torr} pressure_unit;
typedef enum {analogSensor, digitalSensor, sensorFormula} sensor_type;
	
typedef struct sens sensor_s;

void addSensors(int num);
float sensor(int id, pressure_unit unit);
void sensorCalc(int id);
char* sensor_str(int id, pressure_unit unit);
void setCalcFunc(int id, float(*func)(int id));
void deleteSensors();


#endif /* SENSORS_H_ */