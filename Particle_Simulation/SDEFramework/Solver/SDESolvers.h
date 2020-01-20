#pragma once

#ifndef INC_SDESOLVERS_H_
#define INC_SDESOLVERS_H_

// Noise Descriptions for Solvers
#include "../NoiseField.h"
#include "../DoubleNoiseMatrix.h"
#ifdef SIMD_NORMAL_DIST
#include "../NoiseField_SIMD.h"
#endif

//Solvers so far
#include "EulerMaruyama.h"
#include "EulerMaruyama_Normalized.h"
#include "Explicit_Strong_1.h"
#include "Millstein.h"
#include "Heun_NotConsistent.h"
#include "Heun_Strong.h"
#include "WeakTest.h"
#include "Implicit_Midpoint.h"

#ifdef USE_GSL_SOLVERS
#include "Implicit_Midpoint_GSL.h"
#include "Implicit_Midpoint_GSL_Derivative_Free.h"
#endif

#endif //_SDESOLVERS_H_
