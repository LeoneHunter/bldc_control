#pragma once

#include "main.h"
#include "menu.h"
#include "lcd.h"

#ifdef LCD_Class_mode

class Menu_Class
{
	#define NULL_MENU		Null_Menu_Class

	#define PREVIOUS_MENU   (Menu_Class::Selected_Menu_Item->Previous)
	#define NEXT_MENU       (Menu_Class::Selected_Menu_Item->Next)
	#define PARENT_MENU     (Menu_Class::Selected_Menu_Item->Parent)
	#define CHILD_MENU      (Menu_Class::Selected_Menu_Item->Child)

	#define MENU_CREATE(Name, Next, Previous, Parent, Child) \
	extern Menu_Class Next;		\
	extern Menu_Class Previous;	\
	extern Menu_Class Parent;	\
	extern Menu_Class Child;	\
	Menu_Class Name = {&Next, &Previous, &Parent, &Child}



	public:
	Menu_Class(Menu_Class* Next_, Menu_Class* Previous_, Menu_Class* Parent_, Menu_Class* Child_)
	{
		Next		= Next_;
		Previous	= Previous_;
		Parent		= Parent_;
		Child		= Child_;
	};
	
	/* Functions ---------------------------------------------------------*/

	static void Menu_Change(Menu_Class* New_Menu_item)
	{
		extern Menu_Class Null_Menu_Class;

		if (New_Menu_item == &Null_Menu_Class) return;
		
		Selected_Menu_Item = New_Menu_item;
	}

	
	/* Menu Variables ----------------------------------------------------*/
	
	static Menu_Class* Selected_Menu_Item;

	Menu_Class* Next;			
	Menu_Class* Previous;
	Menu_Class* Parent;
	Menu_Class* Child;
	
	

	private:
	
	/* Variables ---------------------------------------------------------*/
	// Prop variables

	u8		page;
	u16		x_pos; 
	u16		y_pos; 
	u16		color; 
	u16		back_color;	
	u8		font_size;

	u8		mode;
	
	char	text1[11];
	char	param1[11];
	char	param2[11];
	char	param3[11];
		
	u32*	data_ptr;
	u8		data_type;
	float	data_multiplier;
};

Menu_Class Null_Menu_Class = {(Menu_Class*)0, (Menu_Class*)0, (Menu_Class*)0, (Menu_Class*)0};
Menu_Class* Menu_Class::Selected_Menu_Item;







MENU_CREATE(Menu_speed4,	Menu_speed5,	NULL_MENU,		NULL_MENU,	NULL_MENU);
MENU_CREATE(Menu_speed5,	Menu_speed6,	Menu_speed4,	NULL_MENU,	NULL_MENU);
MENU_CREATE(Menu_speed6,	NULL_MENU,		Menu_speed5,	NULL_MENU,	NULL_MENU);


void Class_test()
{
	Menu_Class::Selected_Menu_Item = &Menu_speed4;
	

	Menu_Class::Menu_Change(NEXT_MENU);
	Menu_Class::Menu_Change(NEXT_MENU);
	Menu_Class::Menu_Change(NEXT_MENU); 
	
	
}
#endif

#define END				NULL
#define BACKGROUND		WHITE
#define FONTSIZE		2
#define PROPDATAOFFSET	216


enum {IN, OUT};

enum {P_MAIN, P_OPTIONS};


extern u8 appState, ADC_state, sendDataFlag, dataToBeSent;



/******************************************************************************
* MENU
******************************************************************************/	

extern f32					motor_speed_estim;
extern MC_DQSyst			I_DQ;
extern MC_DQSyst			I_DQ_desired;
extern f32					motor_speed_ref;
//extern f32					k1;
extern f32					i_d_pid_p;
extern f32					i_d_pid_i;
extern f32					i_q_pid_p;
extern f32					i_q_pid_i;
extern f32					spd_pid_p;
extern f32					spd_pid_i;
extern PID_params_typedef	IQPIDParams;
extern f32					motorPower;

#define OMEGATORPM			(9.55/7.0)


LCD_Prop_typedef	LCD_Prop_Motor_Cntrl= {P_MAIN,8,25,		BLACK,BACKGROUND,FONTSIZE,TEXT_MODE,	"Motor",		"OFF",		"ON",		"",				(void*)&appState,			TYPE_U8,	1.0,		1.0};
LCD_Prop_typedef	LCD_Prop_Speed		= {P_MAIN,8,45,		BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"Speed",		"",			"",			"",				(void*)&motor_speed_estim,	TYPE_FLOAT,	OMEGATORPM,	1.0};
LCD_Prop_typedef	LCD_Prop_Iq			= {P_MAIN,8,65,		BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"Iq",			"",			"",			"",				(void*)&I_DQ.q,				TYPE_FLOAT,	1.0,		1.0};		
LCD_Prop_typedef	LCD_Prop_Iq_ref		= {P_MAIN,8,85,		BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"Id",			"",			"",			"",				(void*)&I_DQ.d,				TYPE_FLOAT,	1.0,		1.0};	
LCD_Prop_typedef	LCD_Prop_Power		= {P_MAIN,8,105,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"Power",		"",			"",			"",				(void*)&motorPower,		TYPE_FLOAT,	1.0,		1.0};

LCD_Prop_typedef	LCD_Prop_Exec_time	= {P_MAIN,8,125,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"Exe time",		"",			"",			"",				(void*)&exec_time,			TYPE_U32,	1.0,		1.0};		


LCD_Prop_typedef	LCD_Prop_data_send	= {P_MAIN,8,140,	BLACK,BACKGROUND,FONTSIZE,TEXT_MODE,	"Send data",	"send",		"wait",		"",				(void*)&sendDataFlag,		TYPE_U8,	1.0,		1.0};		
LCD_Prop_typedef	LCD_Prop_data_sel	= {P_MAIN,8,160,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"Sel. data",	"Data A",	"Data B",	"Data C",		(void*)&dataToBeSent,	TYPE_U8,	1.0,		1.0};		

LCD_Prop_typedef	LCD_Prop_Options	= {P_MAIN,8,200,	BLACK,BACKGROUND,FONTSIZE,NONE_MODE,	"Options",		"",			"",			"",				NULL,						0,			0,			1.0};		



LCD_Prop_typedef	LCD_Prop_Speed_Set	= {P_OPTIONS,8,25,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"SPEED",		"",			"",			"",				(void*)&motor_speed_ref,	TYPE_FLOAT,	OMEGATORPM,	1.0};		
LCD_Prop_typedef	LCD_Prop_SMO_k1		= {P_OPTIONS,8,45,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"SMO K1",		"",			"",			"",				(void*)&i_d_pid_p,			TYPE_FLOAT,	1.0,		1.0};		
LCD_Prop_typedef	LCD_Prop_i_pid_d_p	= {P_OPTIONS,8,65,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"PID D P",		"",			"",			"",				(void*)&i_d_pid_p,			TYPE_FLOAT,	1.0,		0.01};		
LCD_Prop_typedef	LCD_Prop_i_pid_d_i	= {P_OPTIONS,8,85,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"PID D I",		"",			"",			"",				(void*)&i_d_pid_i,			TYPE_FLOAT,	1.0,		0.01};
LCD_Prop_typedef	LCD_Prop_i_pid_q_p	= {P_OPTIONS,8,105,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"PID Q P",		"",			"",			"",				(void*)&i_q_pid_p,			TYPE_FLOAT,	1.0,		0.01};
LCD_Prop_typedef	LCD_Prop_i_pid_q_i	= {P_OPTIONS,8,125,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"PID Q I",		"",			"",			"",				(void*)&i_q_pid_i,			TYPE_FLOAT,	1.0,		0.01};
LCD_Prop_typedef	LCD_Prop_spd_pid_p	= {P_OPTIONS,8,145,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"PID SPD P",	"",			"",			"",				(void*)&spd_pid_p,			TYPE_FLOAT,	1.0,		0.0001};
LCD_Prop_typedef	LCD_Prop_spd_pid_i	= {P_OPTIONS,8,165,	BLACK,BACKGROUND,FONTSIZE,DATA_MODE,	"PID SPD I",	"",			"",			"",				(void*)&spd_pid_i,			TYPE_FLOAT,	1.0,		0.00001};









/* Menu Functions ---------------------------------------------------------*/
//								NEXT				PREVIOUS			PARENT			CHILD				PROP					FUNC
/* Menu MAIN --------------------------------------------------------------*/

MAKE_MENU(menu_motor_cntrl,		menu_data_send,		menu_options,		NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_Motor_Cntrl,	Key_Action_Toggle);
MAKE_MENU(menu_speed,			NULL_ENTRY,			NULL_ENTRY,			NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_Speed,		Key_Action_Normal);
MAKE_MENU(menu_Iq,				NULL_ENTRY,			NULL_ENTRY,			NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_Iq,			Key_Action_Normal);
MAKE_MENU(menu_Iq_ref,			NULL_ENTRY,			NULL_ENTRY,			NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_Iq_ref,		Key_Action_Normal);
MAKE_MENU(menu_Power,			NULL_ENTRY,			NULL_ENTRY,			NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_Power,		Key_Action_Normal);

MAKE_MENU(menu_adc_state,		NULL_ENTRY,			NULL_ENTRY,			NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_Exec_time,	Key_Action_Normal);


MAKE_MENU(menu_data_send,		menu_data_sel,		menu_motor_cntrl,	NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_data_send,	Key_Action_Toggle);
MAKE_MENU(menu_data_sel,		menu_options,		menu_data_send,		NULL_ENTRY,		NULL_ENTRY,			&LCD_Prop_data_sel,		Key_Action_Modify);

MAKE_MENU(menu_options,			menu_motor_cntrl,	menu_data_sel,		NULL_ENTRY,		menu_speedset,		&LCD_Prop_Options,		Key_Action_Normal);




/* Menu OPTIONS -----------------------------------------------------------*/

MAKE_MENU(menu_speedset,		menu_SMO_k1,			menu_spdpidq,	menu_options,	NULL_ENTRY,			&LCD_Prop_Speed_Set,		Key_Action_Modify);
MAKE_MENU(menu_SMO_k1,			menu_ipiddp,			menu_speedset,	menu_options,	NULL_ENTRY,			&LCD_Prop_SMO_k1,			Key_Action_Modify);
MAKE_MENU(menu_ipiddp,			menu_ipiddi,			menu_SMO_k1,	menu_options,	NULL_ENTRY,			&LCD_Prop_i_pid_d_p,		Key_Action_Modify);
MAKE_MENU(menu_ipiddi,			menu_ipidqp,			menu_ipiddp,	menu_options,	NULL_ENTRY,			&LCD_Prop_i_pid_d_i,		Key_Action_Modify);
MAKE_MENU(menu_ipidqp,			menu_ipidqi,			menu_ipiddi,	menu_options,	NULL_ENTRY,			&LCD_Prop_i_pid_q_p,		Key_Action_Modify);
MAKE_MENU(menu_ipidqi,			menu_spdpidp,			menu_ipidqp,	menu_options,	NULL_ENTRY,			&LCD_Prop_i_pid_q_i,		Key_Action_Modify);
MAKE_MENU(menu_spdpidp,			menu_spdpidq,			menu_ipidqi,	menu_options,	NULL_ENTRY,			&LCD_Prop_spd_pid_p,		Key_Action_Modify);
MAKE_MENU(menu_spdpidq,			menu_speedset,			menu_spdpidp,	menu_options,	NULL_ENTRY,			&LCD_Prop_spd_pid_i,		Key_Action_Modify);


// Menu array
menuItem* menu_items[] = {&menu_speed,		&menu_Iq,			&menu_motor_cntrl,	&menu_options,		&menu_data_send,	&menu_ipidqp,
						  &menu_data_sel,	&menu_adc_state,	&menu_speedset,		&menu_SMO_k1,		&menu_ipiddp,		&menu_ipiddi,	 
						  &menu_ipidqi,		&menu_spdpidp,		&menu_spdpidq,		&menu_Iq_ref,		&menu_Power,
						  END};		






