///---------------------------------------------------------------------------------------------------
// file:		ProblemSelector.h
//
// summary: 	Declares the problem selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_ProblemSelector_H
#define INC_ProblemSelector_H
///---------------------------------------------------------------------------------------------------
#include "Settings/ProblemSettings.h"
#include "AnisotropySelector.h"
#include "BoundarySelector.h"

#pragma once
namespace Selectors
{
	using namespace Settings;
	using namespace Properties;

	///-------------------------------------------------------------------------------------------------
	/// <summary>	A problem type selector. Used to select the Problem and hold
	/// 			the necessary type informations to create the problem</summary>
	///
	/// <seealso cref="T:BasicSelector{ProblemTypeSelector{problem}}"/>
	///-------------------------------------------------------------------------------------------------
	template <IProblem problem>
	class ProblemTypeSelector : public BasicSelector<ProblemTypeSelector<problem>>
	{
		static_assert(problem != IProblem::Problem_undefined, "Compiler is trying to use the wrong version of the ProblemTypeSelector!");
		static_assert(problem != IProblem::Problem_undefined, "The compiler should never try to instatiate a version of this template!");
		//using value_type = IProblem;
		//static constexpr IProblem value = problem;

		//static constexpr IProblem getValue() { return problem; }
	};

}

#endif	// INC_ProblemSelector_H
// end of ProblemSelector.h
///---------------------------------------------------------------------------------------------------
