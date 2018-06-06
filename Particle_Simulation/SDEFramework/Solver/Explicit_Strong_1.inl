/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

//#include "DerivativeFreeMillstein.h" //File is included by header!
//#include <iostream>
namespace SDE_Framework::Solvers
{
	template<typename problem, typename nfield, typename nmatrix>
	BASIC_ALWAYS_INLINE Explicit_Strong_1<problem, nfield, nmatrix>::Explicit_Strong_1(const Settings& SolverSettings, Problem &prob, Precision tstep) :
		GeneralSDESolver<Explicit_Strong_1<problem, nfield, nmatrix>, problem, nfield>(prob, timestep), m_dWgen(1000000, timestep), m_J_j1j2gen(1000000, timestep), m_sqrttimestep(sqrt(timestep))
	{};


	template<typename prob, typename nfield, typename nmatrix>
	BASIC_ALWAYS_INLINE auto Explicit_Strong_1<prob, nfield, nmatrix>::getResultNextFixedTimestep(const DependentType& yi, const IndependentType& xi) const -> ResultType
	{
		const auto& problem = this->m_problem;

		const auto dW{ m_dWgen.getField() };
		const auto J_j1j2{ m_J_j1j2gen.getNoiseMatrix(dW) }; //This calculation can take a long time

		//std::cout << "dW" << dW << std::endl;
		//std::cout << "J_j1j2" << J_j1j2 << std::endl;

		const auto a{ problem.getDeterministicVector(yi,xi) /*+ problem.getDrift(yi)*/ }; //Drift correction not necessary in this Solver!
		const auto b{ problem.getStochasticMatrix(yi) };

		//std::cout << "J_j1j2" << J_j1j2 << std::endl;

		const auto dt = this->m_timestep;
		const auto sqrtdt = this->m_sqrttimestep;

		DependentType tmpvec{ yi + a*dt /*+ b*dW */ };				// Euler-Maruyama Solution without Noise Term

		DependentType tmpvec2{ DependentType::Zero() };		// Additional Corrections

		//Something is wrong with this part ! Could be as simple as a minus sign!
		// Without this part we get the (Ito) Euler-Maruyama solution!
		for (int j1 = Dimensions::NumberOfDependentVariables; j1--;)
		{
			const auto Supportp{ tmpvec + b.col(j1)*sqrtdt };
			const auto Supportm{ tmpvec - b.col(j1)*sqrtdt }; //my changes to the solver

			const auto bSp{ problem.getStochasticMatrix(Supportp) };
			const auto bSm{ problem.getStochasticMatrix(Supportm) };

			const auto difference{ bSp - bSm };
			//const StochasticMatrixType difference{ bSp - b };
			tmpvec2 += difference*(J_j1j2.col(j1)).eval();
			//std::cout <<  "tmpvec2" << (bS - b)*(J_j1j2.row(j1).transpose()) << std::endl;
		};

		//for (int j1 = Dimensions::SizeOfNoiseVector; j1--;) //Running through rows
		//{
		//	DependentType SupportPoint{ tmpvec + b.col(j1).eval()*sqrtdt };
		//	StochasticMatrixType SupportMatrix{ (m_problem)->getStochasticMatrix(SupportPoint) };

		//	for (int j2 = Dimensions::NumberOfDependentVariables; j2--;) // Running through column
		//	{
		//		tmpvec2 += (SupportMatrix - b).eval()*(J_j1j2.row(j1).transpose());
		//	}
		//}

		tmpvec2 /= sqrtdt;
		tmpvec2 += tmpvec + b*dW;
		//std::cout << "Result" << tmpvec2 << std::endl;
		return tmpvec2;
	};
}
