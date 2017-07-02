///---------------------------------------------------------------------------------------------------
// file:		ProblemsFWD.h
//
// summary: 	Forward deklaration of problems
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 07.02.2017

#ifndef INC_ProblemsFWD_H
#define INC_ProblemsFWD_H
///---------------------------------------------------------------------------------------------------
#pragma once


namespace Problems
{
	template<typename precision, typename aniso,bool SimpleModel = false>
	class BrownAndNeelRelaxation;

	template<typename precision, typename aniso>
	class NeelRelaxation;

	template<typename precision, typename aniso>
	class NeelRelaxationSpherical;
}



#endif	// INC_ProblemsFWD_H
// end of ProblemsFWD.h
///---------------------------------------------------------------------------------------------------
