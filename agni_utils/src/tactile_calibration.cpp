/**
 * @file   tactile_calibration.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   2014/03/28
 *
*/

#include <agni_utils/tactile_calibration.hpp>
namespace agni
{
  double tactileCalibration(int raw, int max_adc, TactileSensorType type)
  {
		double cal=0.0;
		int rawclamped=0;
		switch (type)
		{
			case UBI0:
				if (raw >0 && raw < max_adc)
					cal = static_cast<float>(max_adc-raw)/static_cast<float>(max_adc);
				break;
			case Palm:
			case Prox:
			case Middle:
				// clamp raw value to [0; max_adc-1]
				rawclamped=raw;
				if (rawclamped <0) rawclamped=0;
				if (rawclamped >=max_adc) rawclamped=max_adc-1;
				cal =  1/ (1- static_cast<float>(rawclamped)/static_cast<float>(max_adc)) -1;
				if (cal < 0)
					cal=0.0;
				if (cal > 1.0)
					cal = 1.0;
				break;
			default:
				cal = static_cast<float>(raw)/max_adc;
		}
    return cal;
  }
  
  void colorMappingHue(float val, float &r, float &g, float &b)
  {
		//consider val is hue
		if(val>1.0)
			val=1.0;
			
		val*=6;
		int i=(int)val; // integer part
		float f=val-i; //fractionnal part
		if(i==0) {r=1;g=f;b=0;}
		if(i==1) {r=1-f;g=1;b=0;}
		if(i==2) {r=0;g=1;b=f;}
		if(i==3) {r=0;g=1-f;b=1;}
		if(i==4) {r=f;g=0;b=1;}
		if(i==5) {r=1;g=0;b=1-f;}
	}
	
	void colorMapping2Colors(float val, float &col1, float &col2, float th1)
	{
		if(th1==0.0)
			th1=0.01;
		if(th1>=1.0)
			th1=0.99;
		
		float Slope1=1/(th1);
		float Slope2=1/(1-th1);
		if(val<th1)
		{
			col1=Slope1*val;
			col2=0;
		}
		else
		{
			col1=1.0-(val-th1)*Slope2;
			col2=(val-th1)*Slope2;
		}
	}
}
