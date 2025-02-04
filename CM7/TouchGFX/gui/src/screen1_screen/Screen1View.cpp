#include <gui/screen1_screen/Screen1View.hpp>

//#ifndef SIMULATOR
//extern "C"
//{
//	#include "main.h"
//}
//extern volatile int RPM_;

//#endif
Screen1View::Screen1View()
{
	RPM = 1;
}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

//void Screen1View::updateRPMOnGauge(uint32_t rpm)
//{
 //   gauge1.setValue(rpm);   // Set the gauge value to the new RPM
    //gauge1.invalidate();    // Refresh the gauge display
//}

void Screen1View::handleTickEvent()
{
	if(gauge1.getValue() == 0 || gauge1.getValue() == 100)
	{
		RPM *= -1;
	}
	gauge1.updateValue(gauge1.getValue() + RPM, 0);
}
