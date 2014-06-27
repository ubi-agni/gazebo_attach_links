/**
 * @file   tactile_calibration.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   2014/03/28
 *
*/

#ifndef AGNI_TACTILE_CALIBRATION_HPP_
#define AGNI_TACTILE_CALIBRATION_HPP_

namespace agni
{
	enum TactileSensorType {Default = 0,
					 UBI0 = 1,
					 Palm = 2,
					 Prox = 3,
					 Middle = 4
	};
	
	double tactileCalibration(int raw, int max_adc, TactileSensorType type=Default);
	void colorMappingHue(float val, float &r, float &g, float &b);
	void colorMapping2Colors(float val, float &col1, float &col2, float th1=0.33);
}

#endif
