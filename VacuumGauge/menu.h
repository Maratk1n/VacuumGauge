
#ifndef MENU_H_
#define MENU_H_

//#include "LCD.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef enum  {Up, Lower, Left, Right, Ok, Reset} button;

typedef struct menu menu_s;

void createMenusItem(const char *top_text, const char *bottom_text);
void createSubItem(int id, const char *top_text, const char *bottom_text);
void createSub2Item(int id, int subId, const char *top_text, const char *bottom_text);
void clearMenus(void);
void moveMenu(button);
void menuOk(void);
void menuButtton(void);
void setItemAction(int id, void (*act)(button));
void setSubItemAction(int id, int subId, void (*act)(button));
void setSub2ItemAction(int id, int subId, int sub2Id, void (*act)(button));
void setSubItemAsChangeable(int id, int subId);
void setSub2ItemAsChangeable(int id, int subId, int sub2Id);
void setMenuText(int id, char *top_text, char *bottom_text);
void setSubMenuText(int id, int subId, char *top_text, char *bottom_text);
void setSub2MenuText(int id, int subId, int sub2Id, char *top_text, char *bottom_text);
void updateMenu(uint16_t step);
void displayQueue(const char *top_text, const char *bottom_text, uint16_t fix_time_ms);

#endif /* MENU_H_ */