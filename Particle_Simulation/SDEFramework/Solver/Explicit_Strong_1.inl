/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

//#include "DerivativeFreeMillstein.h" //File is included by header!
//#include <iostream>
template<typename problem, typename nfield, typename nmatrix>
BASIC_ALWAYS_INLINE Explicit_Strong_1<problem, nfield, nmatrix>::Explicit_Strong_1(const problem& prob,const Precision& timestep) :
	GeneralSDESolver<Explicit_Strong_1<problem, nfield, nmatrix>,problem,nfield>(prob, timestep), m_dWgen(1000000, timestep), m_J_j1j2gen(1000000, timestep), m_sqrttimestep(sqrt(timestep))
{};


template<typename problem, typename nfield, typename nmatrix>
BASIC_ALWAYS_INLINE auto Explicit_Strong_1<problem, nfield, nmatrix>::getResultNextFixedTimestep(const DependentVectorType& yi, const IndependentVectorType& xi) const -> ResultType
{
	const auto& problem = this->m_problem;

	const DeterministicVectorType dW{ m_dWgen.getField() };
	const StochasticMatrixType J_j1j2{ m_J_j1j2gen.getNoiseMatrix(dW) }; //This calculation can take a long time

	//std::cout << "dW" << dW << std::endl;
	//std::cout << "J_j1j2" << J_j1j2 << std::endl;

	const DeterministicVectorType a{ problem.getDeterministicVector(yi,xi) /*+ problem.getDrift(yi)*/ }; //Drift correction not necessary in this Solver!
	const StochasticMatrixType b{ problem.getStochasticMatrix(yi) };

	//std::cout << "J_j1j2" << J_j1j2 << std::endl;
	
	const auto dt = this->m_timestep;
	const auto sqrtdt = this->m_sqrttimestep;

	DependentVectorType tmpvec{ yi + a*dt /*+ b*dW */ };				// Euler-Maruyama Solution without Noise Term
	
	DependentVectorType tmpvec2{ DependentVectorType::Zero() };		// Additional Corrections

	//Something is wrong with this part ! Could be as simple as a minus sign!
	// Without this part we get the (Ito) Euler-Maruyama solution!
	for(int j1 = Dimensions::NumberOfDependentVariables; j1--;)
	{
		const DeterministicVectorType Supportp{ tmpvec + b.row(j1).transpose()*sqrtdt };
		const DeterministicVectorType Supportm{ tmpvec - b.row(j1).transpose()*sqrtdt }; //my changes to the solver

		const StochasticMatrixType bSp{ problem.getStochasticMatrix(Supportp) };
		const StochasticMatrixType bSm{ problem.getStochasticMatrix(Supportm) };

		const StochasticMatrixType difference{ bSp - bSm };
		//const StochasticMatrixType difference{ bSp - b };
		tmpvec2 += difference*(J_j1j2.col(j1)).eval();
		//std::cout <<  "tmpvec2" << (bS - b)*(J_j1j2.row(j1).transpose()) << std::endl;
	};
	
	//for (int j1 = Dimensions::SizeOfNoiseVector; j1--;) //Running through rows
	//{
	//	DependentVectorType SupportPoint{ tmpvec + b.col(j1).eval()*sqrtdt };
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

