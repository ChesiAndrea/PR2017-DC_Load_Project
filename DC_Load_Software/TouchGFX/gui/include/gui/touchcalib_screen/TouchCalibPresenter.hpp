#ifndef TOUCHCALIBPRESENTER_HPP
#define TOUCHCALIBPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class TouchCalibView;

class TouchCalibPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    TouchCalibPresenter(TouchCalibView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~TouchCalibPresenter() {};

		void saveCalibration(int32_t *p, bool isValid);
		void setLcdBacklight(uint32_t val, int ignoreMode);			

private:
    TouchCalibPresenter();

    TouchCalibView& view;
};

#endif // TOUCHCALIBPRESENTER_HPP
