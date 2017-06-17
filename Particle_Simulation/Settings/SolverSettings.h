///---------------------------------------------------------------------------------------------------
// file:		SolverSettings.h
//
// summary: 	Declares the solver settings class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 25.06.2016

#ifndef INC_SolverSettings_H
#define INC_SolverSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <map>
#include <string>
#include "Archive/NamedValue.h"

namespace Settings
{
	enum class ISolver {Solver_undefined, Solver_EulerMaruyama, Solver_Millstein, Solver_Heun_Strong, Solver_Heun_NotConsistent, Solver_WeakTest, Solver_ExplicitStrong1_0};
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning ( disable : 4592) // Disable VS Debug message
#endif
	const std::map<ISolver, std::string> ISolverMap{ { { ISolver::Solver_undefined,"undefined" },
													   { ISolver::Solver_EulerMaruyama,"EulerMaruyama" },
													   { ISolver::Solver_Millstein,"Millstein" },
													   { ISolver::Solver_Heun_Strong,"Heun_Strong" },
													   { ISolver::Solver_Heun_NotConsistent,"Heun_NotConsistent" },
													   { ISolver::Solver_WeakTest ,"WeakTest" },
													   { ISolver::Solver_ExplicitStrong1_0 ,"ExplicitStrong_1.0"} } };
#ifdef _MSC_VER
#pragma warning (pop)
#endif


	std::string to_string(const ISolver& field);
	template<typename T>
	T from_string(const std::string &);
	template<>
	ISolver from_string<ISolver>(const std::string &string);


	template <typename prec>
	class SolverSettings
	{
		typedef SolverSettings<prec> ThisClass;
	private:
		ISolver			TypeOfSolver{ ISolver::Solver_undefined };
		int32_t			DoubleNoiseApprox { -1 };

	protected:
	public:
		typedef prec							  	Precision;

		explicit SolverSettings(ISolver solver, int32_t doublenoiseapprox) : TypeOfSolver(std::move(solver)), DoubleNoiseApprox(std::move(doublenoiseapprox)) {};
		SolverSettings() {};

		inline const ISolver& getTypeOfSolver() const noexcept  { return TypeOfSolver; };
		inline const int32_t& getDoubleNoiseApprox() const noexcept { return DoubleNoiseApprox; };

		static inline std::string getSectionName() { return std::string{ "Solver_Settings" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			std::string str{ to_string(TypeOfSolver) };
			ar(Archives::createNamedValue(std::string{ "Solver" }, str));
			TypeOfSolver = from_string<decltype(TypeOfSolver)>(str);

			if (TypeOfSolver == ISolver::Solver_ExplicitStrong1_0 || TypeOfSolver == ISolver::Solver_WeakTest)
			{
				ar(Archives::createNamedValue(std::string{ "Approximation_of_double_noise_integral" }, DoubleNoiseApprox));
			}
		}
				
	};
}

#endif	// INC_SolverSettings_H
// end of SolverSettings.h
///---------------------------------------------------------------------------------------------------


