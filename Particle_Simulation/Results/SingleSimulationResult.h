///---------------------------------------------------------------------------------------------------
// file:		Results\SingleSimulationResult.h
//
// summary: 	Declares the single simulation result class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Neumann
// date: 22.10.2016
#pragma once
#ifndef INC_SingleSimulationResult_H
#define INC_SingleSimulationResult_H
///---------------------------------------------------------------------------------------------------

#include <SerAr/Core/NamedValue.h>
#include "MyCEL/basics/BasicMacros.h"

namespace Results
{
	template<typename Simulator>
	class MeanSimulationResult;


	class ISingleSimulationResult
	{
		MY_VIRTUAL_INTERFACE(ISingleSimulationResult)
		ALLOW_DEFAULT_COPY_AND_ASSIGN(ISingleSimulationResult)
		
	public:
	};

	///-------------------------------------------------------------------------------------------------
	/// <summary>	Encapsulates the result of a single simulation. </summary>
	///
	/// <typeparam name="Problem">	Type of the problem which was simulated </typeparam>
	///-------------------------------------------------------------------------------------------------
	template<typename Simulator>
	class SingleSimulationResult : public ISingleSimulationResult
	{
		friend class MeanSimulationResult<Simulator>;


	private:
		using ProblemTraits = typename Simulator::Problem::Traits;
		using FieldTraits = typename Simulator::Field::Traits;
		using Precision = typename Simulator::Problem::Precision;

		using StepResult = typename ProblemTraits::OutputType;
		using Properties = typename ProblemTraits::UsedProperties;


		using ResultType = typename Simulator::OutputVectorType;

		using Times = typename Simulator::StepList;
		using Fields = typename Simulator::FieldVectorList;


		Times							mTimesteps;
		Fields							mField;
		Properties						mProperties;
		StepResult						mWeight{ StepResult::Ones() };
		ResultType						mResult;

	protected:
	public:
		//inline explicit SingleSimulationResult(Properties props, ResultVector result) : mProperties(std::move(props)), mResult(std::move(result)) {};
		//inline explicit SingleSimulationResult(Properties props, ResultVector result, const StepResult& norm) : mProperties(std::move(props)), mResult(std::move(result)), mNormalization(norm) {};
		inline explicit SingleSimulationResult(const Properties &props, const ResultType &result, const Times &time, const Fields& field)
				: mTimesteps(time), mField(field), mProperties(props), mResult(result) {};
		inline explicit SingleSimulationResult(const Properties &props, const ResultType &result, const StepResult& weight, const Times &time, const Fields& field)
			: mTimesteps(time), mField(field), mProperties(props), mWeight(weight), mResult(result) {};

		// Access the Properties
		inline const auto& getProperties(void) const noexcept { return(mProperties); }
		inline void setProperties(const Properties& properties) noexcept { mProperties = properties; }

		// Access the Field
		inline const auto& getField(void) const noexcept { return(mField); }
		inline void setField(const Fields& field) noexcept { mField = field; }

		// Access the Timesteps
		inline const auto& getTimesteps(void) const noexcept { return(mTimesteps); }
		inline void setTimesteps(const Times& timesteps) noexcept { mTimesteps = timesteps; }

		// Access the Result
		inline const auto& getResult(void) const noexcept { return(mResult); }
		inline auto& modifyResult(void) { return(mResult); }
		inline void setResult(const ResultType& result) noexcept { mResult = result; }
		
		// Access the Weight
		inline const auto& getWeight(void) const noexcept { return(mWeight); }
		inline auto& modifyWeight(void) const noexcept  { return(mWeight); }
		inline void setWeight(const StepResult& weight) noexcept { mWeight = weight; }

		template<typename Archive>
		void save(Archive& ar)
		{
			ar(Archives::createNamedValue("UsedProperties", mProperties));
			ar(Archives::createNamedValue("Time", mTimesteps));
			ar(Archives::createNamedValue("Weight", mWeight));
			ar(Archives::createNamedValue("Field", mField));
			ar(Archives::createNamedValue("Results", mResult));
		}

	};

}

#endif	// INC_SingleSimulationResult_H
// end of Results\SingleSimulationResult.h
///---------------------------------------------------------------------------------------------------
