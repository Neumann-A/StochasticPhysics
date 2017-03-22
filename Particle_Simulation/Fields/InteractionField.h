///---------------------------------------------------------------------------------------------------
// file:		InteractionField.h
//
// summary: 	Declares the interaction field class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 02.02.2017

#ifndef INC_InteractionField_H
#define INC_InteractionField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "GeneralField.h"

#include "FieldProperties.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

//TODO: Think about a correct implementation!

template <typename precision, size_t No>
class InteractionField :
	public GeneralField<InteractionField<precision, No>>
{
	using ThisClass = InteractionField<precision, No>;
	using Precision = precision;
	using Base = GeneralField<ThisClass>;
	using Traits = typename Base::Traits;
	using FieldProperties = typename Traits::FieldProperties;
	using FieldVector = typename Traits::FieldVector;

private:
	const prec _prefactor = 1E-7;
	const std::vector<Eigen::Matrix<Precision, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<Precision, 3, 1>>> _distvec;
	std::vector<Precision> _disabs_3;
	std::vector<Precision> _disabs_5;

	std::vector<Eigen::Matrix<prec, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<prec, 3, 1>>> calcDistVector(const Eigen::Matrix<prec, 3, 1> &mypos, std::vector<Eigen::Matrix<prec, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<prec, 3, 1>>> &poslist>)
	{
		std::vector<Eigen::Matrix<prec, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<prec, 3, 1>>> tmp;
		tmp.resize(poslist.size());
		auto tmpit = tmp.begin();
		for (auto it = poslist.begin; it != poslist.end(); ++it, ++tmpit)
		{
			*tmpit = *it - mypos;
		}
		return tmp;
		
	}; 
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	InteractionField(Eigen::Matrix<prec, 3, 1> &mypos ,std::vector<Eigen::Matrix<prec, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<prec, 3, 1>>> &poslist, ) : _posvec(poslist)
	{
		int i = 0;
		for(auto it = this->_posvec.begin )
	}

	~InteractionField()
	{
	}

	FieldVector getField(const std::vector<Eigen::Matrix<prec, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<prec, 3, 1>>> &xi)
	{
	};
};

template<typename prec, size_t no>
class FieldTraits<InteractionField<prec,no>>
{
public:
	using Precision = prec;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, 3, 1>;
	using FieldVectorStdAllocator = Eigen::aligned_allocator<FieldVector>;
};


#endif	// INC_InteractionField_H
// end of InteractionField.h
///---------------------------------------------------------------------------------------------------
