#ifndef TOUCHCALIBVIEW_HPP
#define TOUCHCALIBVIEW_HPP

#include <gui_generated/touchcalib_screen/TouchCalibViewBase.hpp>
#include <gui/touchcalib_screen/TouchCalibPresenter.hpp>

class TouchCalibView : public TouchCalibViewBase
{
public:
	TouchCalibView();
	virtual ~TouchCalibView() {}
	virtual void setupScreen();
	virtual void tearDownScreen();			
	virtual void handleTickEvent();
	virtual void handleClickEvent(const ClickEvent& event);		
	void BackgroundClickHandler(const Box& b, const ClickEvent& e);
	bool isSavedCalibValid;
	Point savedCalid[3];
		
protected:
	void updateCalibrationScreen();
	int CalibrationState;
	int timeoutCounter;
	int CalibrationCounter;
	bool BackgroundPressed;
	Point  startCalibPoints[3];
	Point  newCalibPoints[3];
	int32_t MeasCalibPoints[6];
	bool isNewCalibValid;
	Callback<TouchCalibView, const Box&, const ClickEvent&> BackgroundClickedCallback;
};

#endif // TOUCHCALIBVIEW_HPP
