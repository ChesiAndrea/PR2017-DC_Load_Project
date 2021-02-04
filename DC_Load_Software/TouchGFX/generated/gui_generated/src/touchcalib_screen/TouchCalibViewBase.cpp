/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/touchcalib_screen/TouchCalibViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <texts/TextKeysAndLanguages.hpp>

TouchCalibViewBase::TouchCalibViewBase()
{

    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    __background.setPosition(0, 0, 800, 480);
    __background.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));

    TouchCalib_Background.setPosition(0, 0, 800, 480);
    TouchCalib_Background.setColor(touchgfx::Color::getColorFrom24BitRGB(222, 66, 185));

    CalibPoint_3.setPosition(0, 50, 50, 50);
    CalibPoint_3.setVisible(false);

    line1_1.setPosition(0, 0, 50, 50);
    line1_1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    line1_1.setPainter(line1_1Painter);
    line1_1.setStart(25, 0);
    line1_1.setEnd(25, 50);
    line1_1.setLineWidth(2);
    line1_1.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    CalibPoint_3.add(line1_1);

    line2_1.setPosition(0, 0, 50, 50);
    line2_1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    line2_1.setPainter(line2_1Painter);
    line2_1.setStart(0, 25);
    line2_1.setEnd(50, 25);
    line2_1.setLineWidth(2);
    line2_1.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    CalibPoint_3.add(line2_1);

    CalibPoint_2.setPosition(50, 0, 50, 50);
    CalibPoint_2.setVisible(false);

    line1_2.setPosition(0, 0, 50, 50);
    line1_2Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    line1_2.setPainter(line1_2Painter);
    line1_2.setStart(25, 0);
    line1_2.setEnd(25, 50);
    line1_2.setLineWidth(2);
    line1_2.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    CalibPoint_2.add(line1_2);

    line2_2.setPosition(0, 0, 50, 50);
    line2_2Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    line2_2.setPainter(line2_2Painter);
    line2_2.setStart(0, 25);
    line2_2.setEnd(50, 25);
    line2_2.setLineWidth(2);
    line2_2.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    CalibPoint_2.add(line2_2);

    CalibPoint_1.setPosition(0, 0, 50, 50);
    CalibPoint_1.setVisible(false);

    line1.setPosition(0, 0, 50, 50);
    line1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    line1.setPainter(line1Painter);
    line1.setStart(25, 0);
    line1.setEnd(25, 50);
    line1.setLineWidth(2);
    line1.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    CalibPoint_1.add(line1);

    line2.setPosition(0, 0, 50, 50);
    line2Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    line2.setPainter(line2Painter);
    line2.setStart(0, 25);
    line2.setEnd(50, 25);
    line2.setLineWidth(2);
    line2.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    CalibPoint_1.add(line2);

    textArea_Message.setPosition(100, 0, 600, 25);
    textArea_Message.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    textArea_Message.setLinespacing(0);
    textArea_MessageBuffer[0] = 0;
    textArea_Message.setWildcard(textArea_MessageBuffer);
    textArea_Message.setTypedText(touchgfx::TypedText(T_SINGLEUSEID2));

    add(__background);
    add(TouchCalib_Background);
    add(CalibPoint_3);
    add(CalibPoint_2);
    add(CalibPoint_1);
    add(textArea_Message);
}

void TouchCalibViewBase::setupScreen()
{

}

//Called when the screen is done with transition/load
void TouchCalibViewBase::afterTransition()
{
    //Interaction1
    //When screen is entered execute C++ code
    //Execute C++ code
    int x = HAL::getInstance()->getDisplayWidth();
    int y = HAL::getInstance()->getDisplayHeight();
    CalibPoint_1.setPosition((x*0.1)-25, (y*0.1)-25, 50, 50);
    CalibPoint_2.setPosition((x*0.9)-25, (y*0.5)-25, 50, 50);
    CalibPoint_3.setPosition((x*0.5)-25, (y*0.9)-25, 50, 50);
    textArea_Message.setPosition(100, (y*0.25), 600, 25);
}