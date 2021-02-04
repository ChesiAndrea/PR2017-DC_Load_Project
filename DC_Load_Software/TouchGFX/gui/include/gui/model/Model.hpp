#ifndef MODEL_HPP
#define MODEL_HPP

#include <stdint.h>
#include <touchgfx/events/ClickEvent.hpp>

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
		
	bool getSavedTouchCalibration(int32_t *tcp);
	bool saveTouchCalibration(int32_t *tcp, bool isValid);
		
protected:
    ModelListener* modelListener;
};

#endif // MODEL_HPP
