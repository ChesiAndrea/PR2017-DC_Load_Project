/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef INIT_SCREENVIEWBASE_HPP
#define INIT_SCREENVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/init_screen_screen/Init_ScreenPresenter.hpp>
#include <touchgfx/widgets/Box.hpp>

class Init_ScreenViewBase : public touchgfx::View<Init_ScreenPresenter>
{
public:
    Init_ScreenViewBase();
    virtual ~Init_ScreenViewBase() {}
    virtual void setupScreen();
    virtual void afterTransition();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::Box Init_Screen_Backgroung;

private:

};

#endif // INIT_SCREENVIEWBASE_HPP