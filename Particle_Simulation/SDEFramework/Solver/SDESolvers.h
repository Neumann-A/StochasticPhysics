#pragma once

#ifndef _SDESOLVERS_H_
#define _SDESOLVERS_H_

// Noise Descriptions for Solvers
#include "../NoiseField.h"
#include "../DoubleNoiseMatrix.h"

//Solvers so far
#include "EulerMaruyama.h"
#include "Explicit_Strong_1.h"
#include "Millstein.h"
#include "Heun_NotConsistent.h"
#include "Heun_Strong.h"
#include "WeakTest.h"
#include "Implicit_Midpoint.h"

#endif //_SDESOLVERS_H_

