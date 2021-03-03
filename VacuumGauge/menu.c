#include "menu.h"
#include "LCD.h"

#define UPDATE_TIME 20000 // если в течение UPDATE_TIME мс не будет нажатий, возвращаемся в главное меню

struct menu{
	int size;
	char top_text[16]; //текст меню
	char bottom_text[16]; //текст меню
	menu_s* subMenus;
	void (*action)(button);
	bool isChangeable; //флажок для изменяемых подпунктов
};

static menu_s *menus = NULL;
static int menusCount = 0; //счетчик менюшек
static uint16_t timer = 0;
static int currentMenu = 0; //позиция текущего меню
static int currentSubMenu = -1; //позиция текущего подменю 1 уровня
static int currentSub2Menu = -1; //позиция текущего подменю 2 уровня

// структура для очереди
struct queue_s{
	int current_pos;
	int size;
	char (*top_text)[16];
	char (*bottom_text)[16];
	uint16_t *fix_time;
	uint16_t timer;
};
static struct queue_s queue = {.size = 0, .current_pos = -1, .fix_time = NULL, .top_text = NULL, .bottom_text = NULL, .timer = 0};
static bool queueIsFilling = false;

void createMenusItem(const char *top_text, const char *bottom_text)
{
	menus = (menu_s*)realloc(menus, (++menusCount)*sizeof(menu_s));
	if (menusCount > 1){
		char tmp_top[16] = {"»"};
		char tmp_bottom[16] = {" "};
		strcat(tmp_top, top_text);
		strcat(tmp_bottom, bottom_text);
		memcpy(menus[menusCount-1].top_text, tmp_top, 16);
		memcpy(menus[menusCount-1].bottom_text, tmp_bottom, 16);
	}
	else{
		memcpy(menus[menusCount-1].top_text, top_text, 16);
		memcpy(menus[menusCount-1].bottom_text, bottom_text, 16);
	}
	menus[menusCount-1].size = 0;
	menus[menusCount-1].subMenus = NULL;
	menus[menusCount-1].action = NULL;
	menus[menusCount-1].isChangeable = false;
}

void createSubItem(int id, const char *top_text, const char *bottom_text)
{
	if (id < menusCount){
		menus[id].size++;
		menus[id].subMenus = (menu_s*)realloc(menus[id].subMenus, (menus[id].size)*sizeof(menu_s));
		menus[id].subMenus[menus[id].size-1].size = 0;
		menus[id].subMenus[menus[id].size-1].action = NULL;
		menus[id].subMenus[menus[id].size-1].isChangeable = false;
		menus[id].subMenus[menus[id].size-1].subMenus = NULL;
		memcpy(menus[id].subMenus[menus[id].size-1].top_text, top_text, 16);
		memcpy(menus[id].subMenus[menus[id].size-1].bottom_text, bottom_text, 16);
	}
}

void createSub2Item(int id, int subId, const char *top_text, const char *bottom_text)
{
	if ((id < menusCount) && (subId < menus[id].size)){
		menus[id].subMenus[subId].size++;
		menus[id].subMenus[subId].subMenus = (menu_s*)realloc(menus[id].subMenus[subId].subMenus, (menus[id].subMenus[subId].size)*sizeof(menu_s));
		menus[id].subMenus[subId].subMenus[menus[id].subMenus[subId].size-1].size = 0;
		menus[id].subMenus[subId].subMenus[menus[id].subMenus[subId].size-1].action = NULL;
		menus[id].subMenus[subId].subMenus[menus[id].subMenus[subId].size-1].isChangeable = false;
		menus[id].subMenus[subId].subMenus[menus[id].subMenus[subId].size-1].subMenus = NULL;
		memcpy(menus[id].subMenus[subId].subMenus[menus[id].subMenus[subId].size-1].top_text, top_text, 16);
		memcpy(menus[id].subMenus[subId].subMenus[menus[id].subMenus[subId].size-1].bottom_text, bottom_text, 16);
	}
}
void clearMenus(void)
{
	for(int i = 0; i < menusCount; i++){
		for(int j = 0; j < menus[i].size; j++){
			free(menus[i].subMenus[j].subMenus);
		}
		free(menus[i].subMenus);
	}
	free(menus);
}

void moveMenu(button but)
{
	if (queue.size > 0)
		return;
		
	if (currentMenu > 0) timer = 0;
	switch(but){
		case Up:
			if (currentSub2Menu > -1){
				if (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable && (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action != NULL)){
					menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action(Up);
					break;
				}
				if (currentSub2Menu == 0)
					currentSub2Menu = menus[currentMenu].subMenus[currentSubMenu].size - 1;
				else
					currentSub2Menu--;
			}
			else if (currentSubMenu > -1){
				if (menus[currentMenu].subMenus[currentSubMenu].isChangeable && (menus[currentMenu].subMenus[currentSubMenu].action != NULL)){
					menus[currentMenu].subMenus[currentSubMenu].action(Up);
					break;
				}
				if (currentSubMenu == 0)
					currentSubMenu = menus[currentMenu].size - 1;
				else
					currentSubMenu--;
			}
			else if (currentMenu > 0){
				if (currentMenu == 1)
					currentMenu = menusCount - 1;
				else
					currentMenu--;
			}
			break;
		case Lower:
			if (currentSub2Menu > -1){
				if (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable && (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action != NULL)){
					menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action(Lower);
					break;
				}
				if ((menus[currentMenu].subMenus[currentSubMenu].size - currentSub2Menu) == 1)
					currentSub2Menu = 0;
				else
					currentSub2Menu++;
			}
			else if (currentSubMenu > -1){
				if (menus[currentMenu].subMenus[currentSubMenu].isChangeable && (menus[currentMenu].subMenus[currentSubMenu].action != NULL)){
					menus[currentMenu].subMenus[currentSubMenu].action(Lower);
					break;
				}
				if ((menus[currentMenu].size - currentSubMenu) == 1)
					currentSubMenu = 0;
				else
					currentSubMenu++;
			}
			else if (currentMenu > 0){
				if ((menusCount - currentMenu) == 1)
					currentMenu = 1;
				else
					currentMenu++;
			}
			break;
		case Left:
			break;
		case Right:
			break;
		default:
			break;
	}
	if (currentSub2Menu > -1){
		if (!menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable){
			Display(menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].top_text, menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].bottom_text, 0, 0);
		}
	}
	else if (currentSubMenu > -1){
		if (!menus[currentMenu].subMenus[currentSubMenu].isChangeable){
			Display(menus[currentMenu].subMenus[currentSubMenu].top_text, menus[currentMenu].subMenus[currentSubMenu].bottom_text, 0, 0);
		}
	}
	else if (queue.size == 0){
		Display(menus[currentMenu].top_text, menus[currentMenu].bottom_text, 0, 0);
	}
}

void menuOk(void)
{
	if (queue.size > 0)
		return;
		
	if (currentMenu > 0){
		timer = 0;
		if (currentSubMenu == -1){
			if (menus[currentMenu].action != NULL)
				menus[currentMenu].action(Ok);
			if (menus[currentMenu].size > 0){
				currentSubMenu = 0;
				if (menus[currentMenu].subMenus[currentSubMenu].action != NULL){
					menus[currentMenu].subMenus[currentSubMenu].action(Reset);
				}
			}
			else{
				currentMenu = 0;
			}
		}
		else if ((currentSubMenu > -1) && (currentSub2Menu == -1)){
			if (menus[currentMenu].subMenus[currentSubMenu].action != NULL){
				menus[currentMenu].subMenus[currentSubMenu].action(Ok);
			}
			if (menus[currentMenu].subMenus[currentSubMenu].isChangeable){
				if ((menus[currentMenu].size - currentSubMenu) == 1){
					currentSubMenu = -1;
				}
				else{
					currentSubMenu++;
				}
			}
			else{
				if (menus[currentMenu].subMenus[currentSubMenu].size > 0){
					currentSub2Menu = 0;
				}
				else{
					currentSubMenu = -1;
				}
			}
		}
		else if (currentSub2Menu > -1){
			if (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action != NULL){
				menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action(Ok);
			}
			if (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable){
				if ((menus[currentMenu].subMenus[currentSub2Menu].size - currentSub2Menu) == 1){
					currentSub2Menu = -1;
				}
				else{
					currentSub2Menu++;
				}
			}
			else{
				currentSub2Menu = -1;
			}
		}
	}

	else if (currentSubMenu > -1){
		if (menus[currentMenu].subMenus[currentSubMenu].action != NULL)
			menus[currentMenu].subMenus[currentSubMenu].action(Ok);
		currentSubMenu = -1;
	}
	if (currentSub2Menu > -1){
		if (!menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable){
			Display(menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].top_text, menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].bottom_text, 0, 0);
		}
	}
	else if (currentSubMenu > -1){
		if (!menus[currentMenu].subMenus[currentSubMenu].isChangeable){
			Display(menus[currentMenu].subMenus[currentSubMenu].top_text, menus[currentMenu].subMenus[currentSubMenu].bottom_text, 0, 0);
		}
	}
	else if (queue.size == 0){
		Display(menus[currentMenu].top_text, menus[currentMenu].bottom_text, 0, 0);
	}
}

void menuButtton(void)
{
	if (queue.size > 0)
		return;
		
	if (currentMenu == 0){
		if (menusCount > 0){
			currentMenu = 1;
			timer = 0;
		}
	}
	else if (currentSub2Menu > -1){
		if (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable && (menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action != NULL)){
			menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].action(Reset);
		}
		currentSub2Menu = -1;
		timer = 0;
	}
	else if (currentSubMenu > -1){
		if (menus[currentMenu].subMenus[currentSubMenu].isChangeable && (menus[currentMenu].subMenus[currentSubMenu].action != NULL)){
			menus[currentMenu].subMenus[currentSubMenu].action(Reset); //сбросим все, что натворили
		}
		currentSubMenu = -1;
		timer = 0;
	}
	else if (currentMenu > 0){
		currentMenu = 0;
		timer = UPDATE_TIME;
	}
	if (currentSub2Menu > -1){
		if (!menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable){
			Display(menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].top_text, menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].bottom_text, 0, 0);
		}
	}
	else if (currentSubMenu > -1){
		if (!menus[currentMenu].subMenus[currentSubMenu].isChangeable){
			Display(menus[currentMenu].subMenus[currentSubMenu].top_text, menus[currentMenu].subMenus[currentSubMenu].bottom_text, 0, 0);
		}
	}
	else if (queue.size == 0){
		Display(menus[currentMenu].top_text, menus[currentMenu].bottom_text, 0, 0);
	}
}

void setItemAction(int id, void (*act)(button))
{
	if (id < menusCount){
		menus[id].action = act;
	}
}

void setSubItemAction(int id, int subId, void (*act)(button))
{
	if ((id < menusCount) && (subId < menus[id].size)){
		menus[id].subMenus[subId].action = act;
	}
}

void setSub2ItemAction(int id, int subId, int sub2Id, void (*act)(button))
{
	if ((id < menusCount) && (subId < menus[id].size) && (sub2Id < menus[id].subMenus[subId].size)){
		menus[id].subMenus[subId].subMenus[sub2Id].action = act;
	}
}

void setSubItemAsChangeable(int id, int subId)
{
	if ((id < menusCount) && (subId < menus[id].size)){
		menus[id].subMenus[subId].isChangeable = true;
	}
}

void setSub2ItemAsChangeable(int id, int subId, int sub2Id)
{
	if ((id < menusCount) && (subId < menus[id].size) && (sub2Id < menus[id].subMenus[subId].size)){
		menus[id].subMenus[subId].subMenus[sub2Id].isChangeable = true;
	}
}

void setMenuText(int id, char *top_text, char *bottom_text)
{
	if (id < menusCount){
		if (top_text != NULL){
			if (id > 0){
				char tmp_top[16] = {"»"};
				strcat(tmp_top, top_text);
				memcpy(menus[menusCount-1].top_text, tmp_top, 16);
			}
			else{
				memcpy(menus[id].top_text, top_text, 16);
			}
		}
		if (bottom_text != NULL){
			if (id > 0){
				char tmp_bottom[16] = {" "};
				strcat(tmp_bottom, bottom_text);
				memcpy(menus[menusCount-1].bottom_text, tmp_bottom, 16);
			}
			else{
				memcpy(menus[id].bottom_text, bottom_text, 16);
			}
			
		}
	}
}

void setSubMenuText(int id, int subId, char *top_text, char *bottom_text)
{
	if ((id < menusCount) && (subId < menus[id].size)){
		if (top_text != NULL)
			memcpy(menus[id].subMenus[subId].top_text, top_text, 16);
		if (bottom_text != NULL)
			memcpy(menus[id].subMenus[subId].bottom_text, bottom_text, 16);
	}
}

void setSub2MenuText(int id, int subId, int sub2Id, char *top_text, char *bottom_text)
{
	if ((id < menusCount) && (subId < menus[id].size) && (sub2Id < menus[id].subMenus[subId].size)){
		if (top_text != NULL)
			memcpy(menus[id].subMenus[subId].subMenus[sub2Id].top_text, top_text, 16);
		if (bottom_text != NULL)
			memcpy(menus[id].subMenus[subId].subMenus[sub2Id].bottom_text, bottom_text, 16);
	}
}

void updateMenu(uint16_t step)
{
	//обработка морганий
	static bool blink = true;
	static char borders[3] = {' ', '[', ']'};
	static int blink_timer = 0;
	if (blink_timer < 300){
		blink_timer += step;
	}
	else{
		blink_timer = 0;
		blink = !blink;
	}
	char buf[16] = {borders[blink?1:0]};
		
	timer += timer < UPDATE_TIME? step: 0;

	//работа с очередью
	if (queue.size > 0){
		queue.timer += step;
		if (((queue.timer >= queue.fix_time[queue.current_pos]) || ((queue.fix_time[queue.current_pos] > 30000) && ((queue.size - queue.current_pos) >= 2))) && !queueIsFilling){
			if ((queue.size - queue.current_pos) < 2){
				queue.timer = 0;
				queue.size = 0;
				queue.current_pos = -1;
				free(queue.top_text);
				queue.top_text = NULL;
				free(queue.bottom_text);
				queue.bottom_text = NULL;
				free(queue.fix_time);
				queue.fix_time = NULL;
				timer = UPDATE_TIME;
			}
			else{
				queue.timer = 0;
				queue.current_pos++;
				Display(queue.top_text[queue.current_pos], queue.bottom_text[queue.current_pos], 0, 0);
			}
		}
		return;
	}

	if (timer >= UPDATE_TIME){
		if (currentMenu > 0){
			currentMenu = 0;
			currentSubMenu = -1;
			currentSub2Menu = -1;
		}
		Display(menus[0].top_text, menus[0].bottom_text, 0, 0);
	}
	else if ((currentSub2Menu > -1) && menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].isChangeable){
		strncat(buf, menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].bottom_text, 15);
		strncat(buf, &borders[blink?2:0], 1);
		Display(menus[currentMenu].subMenus[currentSubMenu].subMenus[currentSub2Menu].top_text, buf, 0, 0);
	}
	else if ((currentSubMenu > -1) && menus[currentMenu].subMenus[currentSubMenu].isChangeable){
		strncat(buf, menus[currentMenu].subMenus[currentSubMenu].bottom_text, 15);
		strncat(buf, &borders[blink?2:0], 1);
		Display(menus[currentMenu].subMenus[currentSubMenu].top_text, buf, 0, 0);
	}
}

/* Функция ставит в очередь вывод на дисплей на fix_time_ms миллисекунд
*  Меню будет заблокировано пока очередь не опустошится
*/
void displayQueue(const char *top_text, const char *bottom_text, uint16_t fix_time_ms)
{
	queueIsFilling = true;
	queue.size++;
	queue.top_text = realloc(queue.top_text, sizeof(char[16])*queue.size);
	queue.bottom_text = realloc(queue.bottom_text, sizeof(char[16])*queue.size);
	queue.fix_time = realloc(queue.fix_time, sizeof(uint16_t)*queue.size);
	queue.fix_time[queue.size-1] = fix_time_ms;
	memcpy(queue.top_text[queue.size-1], top_text, 16);
	memcpy(queue.bottom_text[queue.size-1], bottom_text, 16);
	if (queue.current_pos < 0){
		queue.current_pos = 0;
		Display(queue.top_text[queue.current_pos], queue.bottom_text[queue.current_pos], 0, 0);
	}
	queueIsFilling = false;
}
