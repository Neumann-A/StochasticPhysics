///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\WeakTest.inl
//
// summary:	Weak test class
///-------------------------------------------------------------------------------------------------
#pragma once

namespace SDE_Framework
{

	template<typename problem, typename nfield, typename nmatrix>
	inline WeakTest<problem, nfield, nmatrix>::WeakTest(const problem& prob, const Precision& timestep) :
		GeneralSDESolver<WeakTest<problem, nfield, nmatrix>, problem, nfield>(prob, timestep), m_dWgen(1000000, timestep), m_J_j1j2gen(1000000, timestep), m_sqrttimestep(sqrt(timestep))
	{};


	template<typename problem, typename nfield, typename nmatrix>
	inline auto WeakTest<problem, nfield, nmatrix>::getResultNextFixedTimestep(const DependentVectorType& yi, const IndependentVectorType& xi) const -> ResultType
	{
		const auto& problem = this->m_problem;
		const auto dt = this->m_timestep;
		const auto sqrtdt = this->m_sqrttimestep;

		const DeterministicVectorType dW{ m_dWgen.getField() };
		const StochasticMatrixType J_j1j2{ m_J_j1j2gen.getNoiseMatrix(dW) }; //This calculation can take a long time

		const DeterministicVectorType a{ problem.getDeterministicVector(yi,xi) };
		const StochasticMatrixType b{ problem.getStochasticMatrix(yi) };

		const DependentVectorType adt{ a*dt };
		const DependentVectorType tmpvec{ yi + adt };

		const DependentVectorType bdW{ b*dW };
		const DependentVectorType ys{ tmpvec + bdW };
		const DeterministicVectorType as{ problem.getDeterministicVector(ys,xi) };

		DependentVectorType tmpvec2{ DependentVectorType::Zero() };		// Additional Corrections

		for (int j1 = Dimensions::NumberOfDependentVariables; j1--;)
		{
			const DeterministicVectorType Supportp{ tmpvec + b.row(j1).transpose()*sqrtdt };
			const DeterministicVectorType Supportm{ tmpvec - b.row(j1).transpose()*sqrtdt };

			const StochasticMatrixType bSp{ problem.getStochasticMatrix(Supportp) };
			const StochasticMatrixType bSm{ problem.getStochasticMatrix(Supportm) };

			const StochasticMatrixType addition{ bSp + bSm };
			const StochasticMatrixType difference{ bSp - bSm };

			tmpvec2 += difference*(J_j1j2.col(j1)).eval() / sqrtdt;
			tmpvec2 += addition*dW;
		};

		tmpvec2 /= 4.0;
		tmpvec2 += 0.5*bdW;

		const DependentVectorType ys2{ yi + 0.5*as*dt + 0.5*adt + tmpvec2 };

		const DeterministicVectorType as2{ problem.getDeterministicVector(ys2,xi) };

		DependentVectorType Result{ yi + 0.5*as2*dt + 0.5*adt + tmpvec2 };

		return Result;
	};

}