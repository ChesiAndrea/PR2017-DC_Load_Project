#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "iwdg.h"
#include "FlashStorage.h"

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	SET_IWDGFLAG(IWDG_GUITASK);
}

bool Model::getSavedTouchCalibration(int32_t* tcp)
{
	tcp[0] = Storage.touchCalib.Point[0].x;
	tcp[1] = Storage.touchCalib.Point[0].y;
	tcp[2] = Storage.touchCalib.Point[1].x;
	tcp[3] = Storage.touchCalib.Point[1].y;
	tcp[4] = Storage.touchCalib.Point[2].x;
	tcp[5] = Storage.touchCalib.Point[2].y;
	return (FLASHSTORAGE_isValid() && Storage.touchCalib.isValid);
}

bool Model::saveTouchCalibration(int32_t* tcp, bool isValid)
{
	tFLASHSTORAGE_Storage *newStorage = FLASHSTORAGE_Unlock();
	newStorage->touchCalib.isValid = isValid;
	newStorage->touchCalib.Point[0].x = tcp[0];
	newStorage->touchCalib.Point[0].y = tcp[1];
	newStorage->touchCalib.Point[1].x = tcp[2];
	newStorage->touchCalib.Point[1].y = tcp[3];
	newStorage->touchCalib.Point[2].x = tcp[4];
	newStorage->touchCalib.Point[2].y = tcp[5];
	return FLASHSTORAGE_Save();
}
