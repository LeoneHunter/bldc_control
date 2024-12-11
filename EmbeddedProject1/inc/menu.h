#pragma once

/******************************************************************************
* Includes
******************************************************************************/
#include "main.h"
#include "lcd.h"


/******************************************************************************
* Defines
******************************************************************************/

#define Key0			LL_GPIO_PIN_4
#define Key1			LL_GPIO_PIN_3
#define Key2			LL_GPIO_PIN_5
#define Key3			LL_GPIO_PIN_6

#define KEY_NONE		0
#define KEY_DOWN		Key0
#define KEY_UP			Key3
#define KEY_OK			Key2
#define KEY_ESCAPE		Key1

#define KEYMODE_NORMAL	0
#define KEYMODE_MODIFY	1

#define KEYDELAY		8

#define NULL_ENTRY		Null_Menu
#define NO_ACTION		NULL

#define PREVIOUS   ((menuItem*)SelectedMenuItem->Previous)
#define NEXT       ((menuItem*)SelectedMenuItem->Next)
#define PARENT     ((menuItem*)SelectedMenuItem->Parent)
#define CHILD      ((menuItem*)SelectedMenuItem->Child)
#define FUNC	              (SelectedMenuItem->Key_Action)

#define MAKE_MENU(Name, Next, Previous, Parent, Child, Prop, Fcn) \
	extern menuItem Next;		\
	extern menuItem Previous;	\
	extern menuItem Parent;		\
	extern menuItem Child;		\
	menuItem Name = {(void*)&Next, (void*)&Previous, (void*)&Parent, (void*)&Child,  Prop, Fcn}




/******************************************************************************
* Types
******************************************************************************/

typedef struct {
	void				*Next;			// Tree settings
	void				*Previous;
	void				*Parent;
	void				*Child;

	LCD_Prop_typedef	*Prop;			// Corresponding prop
	void	(*Key_Action)(u16);			// Active option
} menuItem;



/******************************************************************************
* Global functions
******************************************************************************/

void Buttons_Init();
u16  IsButtonPressed(uint32_t);
u16  Key_Scan();
void Menu_Change(menuItem*);
void Key_Action_Normal(u16);
void Key_Action_Modify(u16);
void Key_Action_Select(u16);
void Key_Action_Toggle(u16);
void Key_Action_Toggle(u16);
void Key_Menu();








