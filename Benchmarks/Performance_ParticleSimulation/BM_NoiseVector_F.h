///---------------------------------------------------------------------------------------------------
// file:		BM_NoiseVector_F.h
//
// summary: 	Declares the bm noise vector class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 27.07.2017

#ifndef INC_BM_NoiseVector_F_H
#define INC_BM_NoiseVector_F_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "stdafx.h"

#include "SDEFramework/NoiseField.h"


class BM_NoiseVector_F : public ::benchmark::Fixture
{
public:
    using Precision = double;

    template<int dim, typename Generator, typename Distribution>
    using Field = NoiseField<Precision, dim, Generator, Distribution>;

    template<int dim, typename Generator, typename Distribution>
    static auto generateField() { return Field<dim,Generator,Distribution>{ 1'000'000,1E-9 }; }

    BM_NoiseVector_F() = default;
};


#endif	// INC_BM_NoiseVector_F_H
// end of BM_NoiseVector_F.h
///---------------------------------------------------------------------------------------------------
