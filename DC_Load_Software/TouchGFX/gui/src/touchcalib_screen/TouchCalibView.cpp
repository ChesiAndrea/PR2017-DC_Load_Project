#include <gui/touchcalib_screen/TouchCalibView.hpp>
#include <touchgfx/Color.hpp>
#include "touchgfx/transforms/TouchCalibration.hpp"
#include <texts/TextKeysAndLanguages.hpp>
#include "cmsis_os.h"

#define InitTask_GUI_Complete					(0x08)
extern EventGroupHandle_t xCreatedEventGroup;
TouchCalibView::TouchCalibView() : BackgroundClickedCallback(this, &TouchCalibView::BackgroundClickHandler)
{

}

void TouchCalibView::setupScreen()
{
	TouchCalibViewBase::setupScreen();
	int x = HAL::getInstance()->getDisplayWidth();
	int y = HAL::getInstance()->getDisplayHeight();
	
	startCalibPoints[0].x = (int32_t)(x*0.1);
	startCalibPoints[0].y = (int32_t)(y*0.1);

	startCalibPoints[1].x = (int32_t)(x*0.9);
	startCalibPoints[1].y = (int32_t)(y*0.5);

	startCalibPoints[2].x = (int32_t)(x*0.5);
	startCalibPoints[2].y = (int32_t)(y*0.9);
	
	TouchCalib_Background.setClickAction(BackgroundClickedCallback);
	
	CalibrationState = 0;
	CalibrationCounter = 0;
	timeoutCounter = 1800; // 40 sec
	HAL::getInstance()->setFingerSize(1);
	
	xEventGroupSetBits(xCreatedEventGroup, InitTask_GUI_Complete);/* The bits being set. */
	
}

int32_t  mX=1000, qX=0, mY=1000, qY=0;
void TouchCalibView::tearDownScreen()
{
	HAL::getInstance()->setFingerSize(10);
	TouchCalibViewBase::tearDownScreen();
}

void TouchCalibView::updateCalibrationScreen()
{
	switch (CalibrationState)
	{
	case 1:
		CalibPoint_1.setVisible(true);
		CalibPoint_2.setVisible(false);
		CalibPoint_3.setVisible(false);
		Unicode::snprintf(textArea_MessageBuffer, TEXTAREA_MESSAGE_SIZE, TypedText(T_CALIB_PRESS_TOPSX).getText());
		break;

	case 2:
		CalibPoint_1.setVisible(false);
		CalibPoint_2.setVisible(true);
		CalibPoint_3.setVisible(false);
		Unicode::snprintf(textArea_MessageBuffer, TEXTAREA_MESSAGE_SIZE, TypedText(T_CALIB_PRESS_MIDDLEDX).getText());
		break;

	case 3:
		CalibPoint_1.setVisible(false);
		CalibPoint_2.setVisible(false);
		CalibPoint_3.setVisible(true);
		Unicode::snprintf(textArea_MessageBuffer, TEXTAREA_MESSAGE_SIZE, TypedText(T_CALIB_PRESS_BOTTOMCR).getText());
		break;

	case 4:
		CalibPoint_1.setVisible(false);
		CalibPoint_2.setVisible(false);
		CalibPoint_3.setVisible(false);
		if(isNewCalibValid){
			textArea_Message.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
			Unicode::snprintf(textArea_MessageBuffer, TEXTAREA_MESSAGE_SIZE, TypedText(T_CALIB_DONE_SUCCESS).getText());
			mX = (1000 * (startCalibPoints[1].x - startCalibPoints[0].x))/ ( newCalibPoints[1].x - newCalibPoints[0].x); // coefficente angolare
			qX = (1000 * startCalibPoints[0].x) - mX * newCalibPoints[0].x; // offset
			mY = (1000 * (startCalibPoints[2].y - startCalibPoints[0].y))/ ( newCalibPoints[2].y - newCalibPoints[0].y); // coefficente angolare
			qY = (1000 * startCalibPoints[0].y) - mY * newCalibPoints[0].y; // offset
		}
		else{
			textArea_Message.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
			Unicode::snprintf(textArea_MessageBuffer, TEXTAREA_MESSAGE_SIZE, TypedText(T_CALIB_DONE_FAIL).getText());
			mX=1000; 
			qX=0; 
			mY=1000;
			qY=0;
		}
		CalibrationState++;

	default:
		CalibPoint_1.setVisible(false);
		CalibPoint_2.setVisible(false);
		CalibPoint_3.setVisible(false);
		break;
	}
	CalibPoint_1.invalidate();
	CalibPoint_2.invalidate();
	CalibPoint_3.invalidate();
	textArea_Message.invalidate();
}

void TouchCalibView::handleClickEvent(const ClickEvent& event)
{
	TouchCalibViewBase::handleClickEvent(event);
	timeoutCounter = 1800; // 40 sec
}

void TouchCalibView::handleTickEvent()
{
	if (CalibrationState == 0)
	{
		if (!BackgroundPressed)
		{
			if (CalibrationCounter > 1)
				application().gotoMainScreenScreenWipeTransitionEast();
			if(isSavedCalibValid)
			{
				TouchCalibration::setCalibrationMatrix(startCalibPoints, savedCalid);
				application().gotoMainScreenScreenWipeTransitionEast();
			}
			else
			{
				CalibrationState = 1;
				updateCalibrationScreen();
			}
		}
		else
		{			
			CalibrationCounter++;			
			if(CalibrationCounter > 300)
				Unicode::snprintf(textArea_MessageBuffer, TEXTAREA_MESSAGE_SIZE, TypedText(T_CALIB_START_RELEASE).getText());
			else
				Unicode::snprintf(textArea_MessageBuffer, TEXTAREA_MESSAGE_SIZE, "%02d", CalibrationCounter);
			textArea_Message.invalidate();
		}
	}
	
	timeoutCounter--;
	if(timeoutCounter <= 0){
		application().gotoMainScreenScreenWipeTransitionEast();
	}
}

void TouchCalibView::BackgroundClickHandler(const Box& b, const ClickEvent& evt)
{
	if (evt.getType() == ClickEvent::PRESSED)
	{
		BackgroundPressed = true;
	}

	if (evt.getType() == ClickEvent::RELEASED)
	{
		if (CalibrationState == 0)
		{
			if (CalibrationCounter > 300)
				CalibrationState = 1;
		}
		else
		{
			if (CalibrationState == 5)
			{
				if (isNewCalibValid) presenter->saveCalibration(MeasCalibPoints, isNewCalibValid);
				application().gotoMainScreenScreenWipeTransitionEast();
			}
			else
			{
				MeasCalibPoints[(CalibrationState*2) - 2] = evt.getX();
				MeasCalibPoints[(CalibrationState*2) - 1] = evt.getY();
				CalibrationState++;
				
				if(CalibrationState == 4){
					int pixelTolerance = 150;
					isNewCalibValid = true;
					for (int i = 0; i<3; i++)
					{
						isNewCalibValid &= ((abs<int>(MeasCalibPoints[(i*2)]-startCalibPoints[i].x)) <= pixelTolerance);
						isNewCalibValid &= ((abs<int>(MeasCalibPoints[(i*2)+1]-startCalibPoints[i].y)) <= pixelTolerance);
						newCalibPoints[i].x = MeasCalibPoints[(i*2)];
						newCalibPoints[i].y = MeasCalibPoints[(i*2)+1];
					}
				}
			}
		}
		BackgroundPressed = false;
	}
	updateCalibrationScreen();
}
