///---------------------------------------------------------------------------------------------------
// file:		MeanSimulationResult.h
//
// summary: 	Declares the mean simulation result class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 22.10.2016

#ifndef INC_MeanSimulationResult_H
#define INC_MeanSimulationResult_H
///---------------------------------------------------------------------------------------------------
#pragma once

#endif	// INC_MeanSimulationResult_H
// end of MeanSimulationResult.h
///---------------------------------------------------------------------------------------------------
#include <algorithm>
#include <functional>
#include <exception>

#include "basics/BasicMacros.h"
#include "Archive/NamedValue.h"

#include "SingleSimulationResult.h"

namespace Results
{
	class IMeanSimulationResult
	{
		MY_VIRTUAL_INTERFACE(IMeanSimulationResult)
	public:
		virtual IMeanSimulationResult& operator+=(const ISingleSimulationResult&) = 0;
	};

	template<typename Simulator>
	class MeanSimulationResult : public IMeanSimulationResult
	{
	private:
		using ProblemTraits = typename Simulator::Problem::Traits;
		using FieldTraits = typename Simulator::Field::Traits;
		using Precision = typename Simulator::Problem::Precision;

		using StepResult = typename ProblemTraits::OutputType;
		using Properties = typename ProblemTraits::UsedProperties;


		using ResultType = typename Simulator::OutputVectorType;

		using Times = typename Simulator::StepList;
		using Fields = typename Simulator::FieldVectorList;

		using ConcreteSingleResultType = typename Simulator::Traits::SingleResultType;

		StepResult					  mWeight{ StepResult::Zero() };
		Times						  mTimesteps;
		Fields						  mField;
		ResultType					  mResult;
		Properties					  mProperties;
		std::size_t					  mNumberOfSingleResults{ 0 };

		void normalize() 
		{
			if (!(mWeight == StepResult::Zero())) //We dont normalize if our normalization is 0
			{
				for (auto& elem : mResult)
				{
					elem = elem.cwiseQuotient(mWeight);
				}
			}

			if(mNumberOfSingleResults >= 1)
				mProperties /= mNumberOfSingleResults;
			}

	public:
		MeanSimulationResult& operator+=(const ISingleSimulationResult& irhs) override final
		{
			assert( (dynamic_cast<ConcreteSingleResultType const *>(&irhs)!= nullptr) );

			const auto& rhs = reinterpret_cast<const ConcreteSingleResultType&>(irhs); //We use reinterpretcast here because it is save and we now the type!
			
			return (*this += rhs);
		}

		inline MeanSimulationResult& operator+=(const ConcreteSingleResultType& rhs)
		{
			const auto& tmp{ rhs.getResult() };
			const auto& weight{ rhs.getWeight()};
			
			//Add single weight to mean Weight
			mWeight += weight;

			//If empty just copy the data from the single result to the mean result and return
			if (mResult.empty())
			{
				mProperties = rhs.getProperties();
				mTimesteps = rhs.getTimesteps();
				mField = rhs.getField();
				mResult = tmp;
				std::for_each(mResult.begin(), mResult.end(), [&weight](auto& elem1) { elem1 = elem1.cwiseProduct(weight); });
				mWeight = weight;
				mNumberOfSingleResults = 1;
				return *this;
			}

			//Just to be sure!
			assert(mResult.size() == tmp.size());

			//Add new single result to mean result with correct weight!
			const auto weightplus = [&weight](const auto& elem1, const auto &elem2) { return (elem1 + elem2.cwiseProduct(weight)); };
			std::transform(mResult.cbegin(), mResult.cend(), tmp.cbegin(), mResult.begin(), weightplus);
			
			mProperties += rhs.getProperties();
			mNumberOfSingleResults++;

			return *this;
		}

		void setTimesteps(Times &Times)
		{
			std::swap(mTimesteps, Times);
		}

		void setFields(Fields &Fields)
		{
			std::swap(mField, Fields);
		}

		template<typename Archive>
		void save(Archive& ar)
		{
			normalize(); //Normalize Data! To get Mean results;

			ar(Archives::createNamedValue("Time", mTimesteps));
			ar(Archives::createNamedValue("Field", mField));
			ar(Archives::createNamedValue("Results", mResult));
			ar(Archives::createNamedValue("Weight", mWeight));
			ar(Archives::createNamedValue("Mean_Properties", mProperties));
		}

	};
}
