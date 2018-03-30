/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

//#include "SinusoidalField.h" // File is inclued by header!
//
//#define _USE_MATH_DEFINES
//#include <cmath> // For M_PI

#include <cmath>
#include "math/math_constants.h"
//
////Constructor
//template<typename precision>
//SinusoidalField<precision>::SinusoidalField(const FieldParameters& params)
//	: GeneralField<precision, SinusoidalField<precision>, 3>(), _params(params),
//	_angularfrequency(2 * m_pi *(*params.getFrequencies().begin())),
//	_phase(*params.getPhases().begin()), _offset(*params.getAmplitudes().begin()),
//	_ampDirection(*(++(params.getAmplitudes().begin())))
//{
//	//assert(_angularfrequency > 0), "Sinusoidal Field Frequency should be greater than zero!"));
//	//_period = 2 * M_PI / _angularfrequency;
//};

//Constructor
template<typename precision>
SinusoidalField<precision>::SinusoidalField(const FieldProperties& params)
	: GeneralField<SinusoidalField<precision>>(), /*_params(params),*/
	_angularfrequency(math::constants::two_pi<precision> * params.getFrequencies().at(0)),
	_phase(params.getPhases().at(0)), _ampDirection(params.getAmplitudes().at(1)), _offset(params.getAmplitudes().at(0))
	
{
	//assert(_angularfrequency > 0), "Sinusoidal Field Frequency should be greater than zero!"));
	//_period = 2 * M_PI / _angularfrequency;
};

//Getter for the field Value; actual function is defined in the constructor
template<typename precision>
inline typename SinusoidalField<precision>::FieldVector SinusoidalField<precision>::getField(const precision time)  const
{
	const precision sinwt = std::sin(_angularfrequency*time + _phase);
	return (_ampDirection * sinwt +_offset).eval();
}

 
