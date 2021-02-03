#ifndef INIT_SCREENVIEW_HPP
#define INIT_SCREENVIEW_HPP

#include <gui_generated/init_screen_screen/Init_ScreenViewBase.hpp>
#include <gui/init_screen_screen/Init_ScreenPresenter.hpp>

class Init_ScreenView : public Init_ScreenViewBase
{
public:
    Init_ScreenView();
    virtual ~Init_ScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // INIT_SCREENVIEW_HPP
