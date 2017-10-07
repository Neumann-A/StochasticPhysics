///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\WeakTest.inl
//
// summary:	Weak test class
///-------------------------------------------------------------------------------------------------
#pragma once

namespace SDE_Framework::Solvers
{

	template<typename problem, typename nfield, typename nmatrix>
	inline WeakTest<problem, nfield, nmatrix>::WeakTest(const Settings& SolverSettings, Problem &prob, Precision tstep) :
		GeneralSDESolver<WeakTest<problem, nfield, nmatrix>, problem, nfield>(prob, timestep), m_dWgen(1000000, timestep), m_J_j1j2gen(1000000, timestep), m_sqrttimestep(sqrt(timestep))
	{};


	template<typename problem, typename nfield, typename nmatrix>
	inline auto WeakTest<problem, nfield, nmatrix>::getResultNextFixedTimestep(const DependentType& yi, const IndependentType& xi) const -> ResultType
	{
		const auto& problem = this->m_problem;
		const auto dt = this->m_timestep;
		const auto sqrtdt = this->m_sqrttimestep;

		const auto dW{ m_dWgen.getField() };
		const auto J_j1j2{ m_J_j1j2gen.getNoiseMatrix(dW) }; //This calculation can take a long time

		const auto a{ problem.getDeterministicVector(yi,xi) };
		const auto b{ problem.getStochasticMatrix(yi) };

		const auto adt{ a*dt };
		const auto tmpvec{ yi + adt };

		const auto bdW{ b*dW };
		const auto ys{ tmpvec + bdW };
		const auto as{ problem.getDeterministicVector(ys,xi) };

		DependentType tmpvec2{ DependentType::Zero() };		// Additional Corrections

		for (int j1 = Dimensions::NumberOfDependentVariables; j1--;)
		{
			const auto Supportp{ tmpvec + b.col(j1)*sqrtdt };
			const auto Supportm{ tmpvec - b.col(j1)*sqrtdt };

			const auto bSp{ problem.getStochasticMatrix(Supportp) };
			const auto bSm{ problem.getStochasticMatrix(Supportm) };

			const auto addition{ bSp + bSm };
			const auto difference{ bSp - bSm };

			tmpvec2 += difference*(J_j1j2.col(j1)).eval() / sqrtdt;
			tmpvec2 += addition*dW;
		};

		tmpvec2 /= 4.0;
		tmpvec2 += 0.5*bdW;

		const DependentType ys2{ yi + 0.5*as*dt + 0.5*adt + tmpvec2 };

		const DeterministicType as2{ problem.getDeterministicVector(ys2,xi) };

		DependentType Result{ yi + 0.5*as2*dt + 0.5*adt + tmpvec2 };

		return Result;
	};

}