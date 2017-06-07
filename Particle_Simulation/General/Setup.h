#pragma once

#include "GeneralDefines.h"
#include "basics/GlobalParameters.h"

#ifdef _MSC_VER
#pragma warning(push) 
// Warnings are deactivated for /Wall (does not work with all warnings because some warnings are issued after this deactivation! So compiling with /Wall is not really an option)
// It is better to raise the Warning level by hand before your own code and lower it afterwards! This circumenvents late warnings due to the standard library. 
//#include <codeanalysis/warnings.h>
//#pragma warning(disable: ALL_CODE_ANALYSIS_WARNINGS) // Dont deal with warnings which are not within our code!
#pragma warning(disable: 4100 4127 4244 4251 4350 4365 4371 4503 4512 4514 4619 4640 4668 4714 4820)  //Dont deal with warnings which are not within our code!
#endif

//Deactivate unimportant warnings
//#pragma warning (disable: 4514) //removal of not used inline functions
//#pragma warning (disable: 4710) //function is not inline

#include <cmath>				//for general math
#include <iostream>				//streams
#include <fstream>				// File streams
#include <chrono>				//for timing
#include <memory>
#include <thread>
#include <array>
#include <vector>
#include <random>		
#include <functional>
#include <regex>
#include <typeinfo>
#include <experimental/filesystem>

#include <Eigen/Core>			// for more vector/matrix math
#include <Eigen/StdVector>		// for Eigen std::vector allocator
#include <Eigen/Geometry>		// for cross product


#include "../Basic_Library/basics/BasicMacros.h"
#include "../Basic_Library/basics/ThreadManager.h"
#include "../Basic_Library/basics/Timer.h"
#include "CommandOptions.h"

#include "Simulator/ISingleParticleSimulator.h"
#include "Simulator/SingleParticleSimulator.h"
#include "Fields/SinusoidalField.h"
#include "Fields/LissajousField.h"
#include "Problems/Problems.h"
#include "SDEFramework/Solver/SDESolvers.h"
#include "Problems/Anisotropy/UniaxialAnisotropy.h"

#define ANISOTROPY UniaxialAnisotropy<PREC>

//#define PROBLEM NeelRelaxation<PREC, ANISOTROPY> // Describes the Problem

#define PROBLEM Problems::BrownAndNeelRelaxation<PREC,ANISOTROPY>

#define DIMS PROBLEM::Dimension // Defines the dimensions of the vectors
#define PARPARAMS PROBLEM::Parameters

#define FIELD SinusoidalField<PREC>


#ifdef USE_BOOST
#include <boost/random/mersenne_twister.hpp>
#ifndef USE_PCG_RANDOM
#define NOISEFIELD NoiseField<PREC, DIMS::NumberOfDependentVariables, boost::random::mt19937_64> // Describes the Random Noise Field
#define DOUBLENOISEMATRIX DoubleNoiseMatrix<PREC, -1, DIMS::SizeOfNoiseVector, boost::random::mt19937_64>
#else
#include <pcg_random.hpp>
#define NOISEFIELD NoiseField<PREC, DIMS::NumberOfDependentVariables, pcg64_k1024_fast> // Describes the Random Noise Field
#define DOUBLENOISEMATRIX DoubleNoiseMatrix<PREC, -1, DIMS::SizeOfNoiseVector, pcg64_k1024_fast>
#endif
#else
#ifdef USE_PCG_RANDOM
#include <pcg_random.hpp>
#define NOISEFIELD NoiseField<PREC, DIMS::NumberOfDependentVariables, pcg64_k1024_fast> // Describes the Random Noise Field
#define DOUBLENOISEMATRIX DoubleNoiseMatrix<PREC, -1, DIMS::SizeOfNoiseVector, pcg64_k1024_fast>
#else
#define NOISEFIELD NoiseField<PREC, DIMS::NumberOfDependentVariables, std::mt19937_64> // Describes the Random Noise Field
#define DOUBLENOISEMATRIX DoubleNoiseMatrix<PREC, 0, DIMS::SizeOfNoiseVector, std::mt19937_64>
#endif
#endif


#define SOLVER // Describes the Solver and Manages that the data is stored

#define SOLVERMETHOD EulerMaruyama<PROBLEM, NOISEFIELD> // Describes the solving Method
//Simple model 73,6s vs 84s Full model

/*#define SOLVERMETHOD DerivativeFreeMillstein<PROBLEM, NOISEFIELD, DOUBLENOISEMATRIX>*/ // Describes the solving Method

// p=-1 0.1 GSteps t=222s
// p= 2 0.1 GSteps t=431s
// p= 4 0.1 GSteps t=584s
// p= 6 0.1 GSteps t=745s

#define SIMULATOR SingleParticleSimulator<SOLVERMETHOD,FIELD>

#define NumIntThreads 8 // Number of internal Threads used within one core
#define NumExtCores 8 // Number of cores to run 

#ifdef _MSC_VER
#pragma warning(pop) // reactivate all warnings
#endif