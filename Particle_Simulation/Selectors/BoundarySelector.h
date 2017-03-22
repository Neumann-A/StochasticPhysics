///---------------------------------------------------------------------------------------------------
// file:		BoundarySelector.h
//
// summary: 	Declares the boundary selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_BoundarySelector_H
#define INC_BoundarySelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "BasicSelector.h"

namespace Selectors
{
	enum class IBoundary {};

	template <IBoundary Boundary>
	class BoundarySelector : public BasicSelector<BoundarySelector<Boundary>> {};

}


#endif	// INC_BoundarySelector_H
// end of BoundarySelector.h
///---------------------------------------------------------------------------------------------------
