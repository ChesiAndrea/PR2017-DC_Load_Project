#ifndef INIT_SCREENPRESENTER_HPP
#define INIT_SCREENPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Init_ScreenView;

class Init_ScreenPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Init_ScreenPresenter(Init_ScreenView& v);

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

    virtual ~Init_ScreenPresenter() {};

private:
    Init_ScreenPresenter();

    Init_ScreenView& view;
};

#endif // INIT_SCREENPRESENTER_HPP