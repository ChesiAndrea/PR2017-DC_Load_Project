#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "iwdg.h"

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	SET_IWDGFLAG(IWDG_GUITASK);
}
