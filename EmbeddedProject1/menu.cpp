#include "menu.h"

/******************************************************************************
* Global Variables
******************************************************************************/

menuItem	Null_Menu			= {NULL, NULL, NULL, NULL, NULL, NULL};

menuItem*	SelectedMenuItem	= &Null_Menu; 
menuItem*	PreviousMenuItem	= &Null_Menu; 

u8			KEYMODE				= KEYMODE_NORMAL;




/******************************************************************************
* Global functions
******************************************************************************/

void Buttons_Init()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin		= LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode	= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull	= LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

u16  IsButtonPressed(uint32_t PinMask)
{
	return (u16)(READ_BIT(GPIOE->IDR, PinMask) == (PinMask));
}

u16  Key_Scan()
{
	u8	Button = KEY_NONE;

	if	   (!IsButtonPressed(KEY_UP))		Button = KEY_UP;
	else if(!IsButtonPressed(KEY_DOWN))		Button = KEY_DOWN;
	else if(!IsButtonPressed(KEY_OK))		Button = KEY_OK;
	else if(!IsButtonPressed(KEY_ESCAPE))	Button = KEY_ESCAPE;

	return Button;
}

void Menu_Change(menuItem* NewMenu)
{
	if ((void*)NewMenu == (void*)&NULL_ENTRY)	return;
	PreviousMenuItem = SelectedMenuItem;	
	SelectedMenuItem = NewMenu;	
}



void Key_Action_Normal(u16 key)
{
	switch (key)											
	{
		case KEY_NONE:		return;		
		case KEY_UP:		Menu_Change(PREVIOUS);	break;	
		case KEY_DOWN:		Menu_Change(NEXT);		break;			
		case KEY_OK:		Menu_Change(CHILD);		break;			
		case KEY_ESCAPE:	Menu_Change(PARENT);	break;	
	}
}

void Key_Action_Modify(u16 key)
{
	static  u8  previouskey	= KEY_NONE;	
	static  f32 increment;

	if (key != KEY_NONE && key == previouskey)	
	{
		increment  += SelectedMenuItem->Prop->data_increment;
	}		
	else
	{
		increment	= SelectedMenuItem->Prop->data_increment;
		previouskey = key;
	}

	switch(KEYMODE)
	{
		case KEYMODE_NORMAL:
			switch (key)											
			{
				case KEY_NONE:		return;		
				case KEY_UP:		Menu_Change(PREVIOUS);			break;	
				case KEY_DOWN:		Menu_Change(NEXT);				break;			
				case KEY_OK:		KEYMODE = KEYMODE_MODIFY;		break;			
				case KEY_ESCAPE:	Menu_Change(PARENT);			break;	
			}														break;		
		case KEYMODE_MODIFY:
			switch (key)											
			{
				case KEY_NONE:		return;		
				case KEY_UP:		
				switch(SelectedMenuItem->Prop->data_type)
				{
					case TYPE_U8:		(*( u8*)SelectedMenuItem->Prop->data_ptr) += increment; break;
					case TYPE_U16:		(*(u16*)SelectedMenuItem->Prop->data_ptr) += increment; break;
					case TYPE_U32:		(*(u32*)SelectedMenuItem->Prop->data_ptr) += increment; break;
					case TYPE_S8:		(*( s8*)SelectedMenuItem->Prop->data_ptr) += increment; break;
					case TYPE_S16:		(*(s16*)SelectedMenuItem->Prop->data_ptr) += increment; break;
					case TYPE_S32:		(*(s32*)SelectedMenuItem->Prop->data_ptr) += increment; break;
					case TYPE_FLOAT:	(*(f32*)SelectedMenuItem->Prop->data_ptr) += increment; break;
					case TYPE_DOUBLE:	(*(d64*)SelectedMenuItem->Prop->data_ptr) += increment; break;
				}													break;				
				case KEY_DOWN:		
				switch(SelectedMenuItem->Prop->data_type)
				{
					case TYPE_U8:		(*( u8*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
					case TYPE_U16:		(*(u16*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
					case TYPE_U32:		(*(u32*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
					case TYPE_S8:		(*( s8*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
					case TYPE_S16:		(*(s16*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
					case TYPE_S32:		(*(s32*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
					case TYPE_FLOAT:	(*(f32*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
					case TYPE_DOUBLE:	(*(d64*)SelectedMenuItem->Prop->data_ptr) -= increment; break;
				}													break;
				case KEY_OK:										break;			
				case KEY_ESCAPE:	KEYMODE = KEYMODE_NORMAL;		break;	
			}														break;
	}
}

void Key_Action_Select(u16 key)
{
	//static	u8 KEYMODE	= KEYMODE_NORMAL;	

	switch(KEYMODE)
	{
		case KEYMODE_NORMAL:
			switch (key)											
			{
				case KEY_NONE:		return;		
				case KEY_UP:		Menu_Change(PREVIOUS);			break;	
				case KEY_DOWN:		Menu_Change(NEXT);				break;			
				case KEY_OK:		KEYMODE = KEYMODE_MODIFY;		break;			
				case KEY_ESCAPE:	Menu_Change(PARENT);			break;	
			}														break;		
		case KEYMODE_MODIFY:
			switch (key)											
			{
				case KEY_NONE:		return;		
				case KEY_UP:		if   (*(u8*)SelectedMenuItem->Prop->data_ptr == 2)	return;		
									else (*(u8*)SelectedMenuItem->Prop->data_ptr)++;	
																	break;	
				case KEY_DOWN:		if	 (*(u8*)SelectedMenuItem->Prop->data_ptr == 0)	return;		
									else (*(u8*)SelectedMenuItem->Prop->data_ptr)--;
																	break;			
				case KEY_OK:										break;			
				case KEY_ESCAPE:	KEYMODE = KEYMODE_NORMAL;		break;	
			}														break;
	}
}

void Key_Action_Toggle(u16 key)
{
	switch (key)											
	{
		case KEY_NONE:		return;		
		case KEY_UP:		Menu_Change(PREVIOUS);						break;	
		case KEY_DOWN:		Menu_Change(NEXT);							break;			
		case KEY_OK:		*(u8*)SelectedMenuItem->Prop->data_ptr ^= 1;break;			
		case KEY_ESCAPE:	Menu_Change(PARENT);						break;	
	}
}



void Key_Menu()
{
	u8			Button				= KEY_NONE;
	static u8	PreviousButton		= KEY_NONE;
	static u32	KeyPressedCounter	= 0;

	Button = Key_Scan();

	if (Button != KEY_NONE && Button == PreviousButton && KeyPressedCounter != KEYDELAY)// Если кнопка удерживается, считаем циклы
	{			
		KeyPressedCounter++;		
		return;				
	}		
	
	if (Button != PreviousButton)	// Если кнопка отпущена, обнуляем счетчик
	{
		PreviousButton = Button;
		KeyPressedCounter = 0;
	}

	SelectedMenuItem->Key_Action(Button);
}
