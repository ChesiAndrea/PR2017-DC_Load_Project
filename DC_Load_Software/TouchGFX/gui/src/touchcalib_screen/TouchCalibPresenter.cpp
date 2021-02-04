#include <gui/touchcalib_screen/TouchCalibView.hpp>
#include <gui/touchcalib_screen/TouchCalibPresenter.hpp>

TouchCalibPresenter::TouchCalibPresenter(TouchCalibView& v)
    : view(v)
{

}

void TouchCalibPresenter::activate()
{
	int32_t tcp[6];
	view.isSavedCalibValid = model->getSavedTouchCalibration(tcp);
	view.savedCalid[0].x = tcp[0];
	view.savedCalid[0].y = tcp[1];
	view.savedCalid[1].x = tcp[2];
	view.savedCalid[1].y = tcp[3];
	view.savedCalid[2].x = tcp[4];
	view.savedCalid[2].y = tcp[5];
}

void TouchCalibPresenter::deactivate()
{

}

void TouchCalibPresenter::saveCalibration(int32_t *p, bool isValid)
{
	model->saveTouchCalibration(p, isValid);
}


