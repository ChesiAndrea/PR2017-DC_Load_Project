#include <gui/main_screen_screen/Main_ScreenView.hpp>
#include "cmsis_os.h"

Main_ScreenView::Main_ScreenView()
{

}

extern EventGroupHandle_t xCreatedEventGroup;
#define InitTask_GUI_Complete		(0x08)
void Main_ScreenView::setupScreen()
{
    Main_ScreenViewBase::setupScreen();
		xEventGroupSetBits(xCreatedEventGroup, InitTask_GUI_Complete);/* The bits being set. */
}

void Main_ScreenView::tearDownScreen()
{
    Main_ScreenViewBase::tearDownScreen();
}
