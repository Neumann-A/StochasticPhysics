///-------------------------------------------------------------------------------------------------
// file:	BrownianMotion.h
//
// summary:	Declares the brownian motion class
///-------------------------------------------------------------------------------------------------

#pragma once

#include "GeneralSDEProblem.h"

#include "BrownianMotionParameters.h"
#include "BoundaryCondition.h"

///-------------------------------------------------------------------------------------------------
/// <summary>	Defines the Dimension of the related Vectors. </summary>
///
/// <remarks>	Neumann, 07.07.2016. </remarks>
///-------------------------------------------------------------------------------------------------
const struct BrownianMotionDimension : GeneralSDEDimension<3, 3, 3> //thats pretty handy
{ } BrownianMotionDimensionVar; //too get the memory space (else the compiler will optimize the infnormation away; TODO: use traits templated class instead!)

///-------------------------------------------------------------------------------------------------
/// <summary>	Definition for the Brownian Motion Problem </summary>
///
/// <remarks>	Neumann, 07.07.2016. </remarks>
///
/// <typeparam name="prec">	   	Type of Precission (Float, Double). </typeparam>
/// <typeparam name="force">   	Type of the force. </typeparam>
/// <typeparam name="boundary">	Type of the boundary condition. </typeparam>
///-------------------------------------------------------------------------------------------------
template <typename prec, typename boundary>
class BrownianMotion : public GeneralSDEProblem<prec, BrownianMotion<prec, boundary>, BrownianMotionDimension>
{
public:
	typedef prec																						   Precision;
	typedef BrownianMotionDimension																		   Dimension;
	typedef typename Problems::BrownianMotionParameters<Precision>										   Parameters;
	//typedef force																						   ExternalForce;
	typedef boundary																					   BoundaryCondition;

	/*Better use a extra trait template class with the according typedefs!*/
	typedef Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::SizeOfNoiseVector> StochasticMatrixType;
	typedef Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, 1>							  DeterministicType;
	typedef DeterministicType																		  DependentType;
	typedef Eigen::Matrix<Precision, Dimension::NumberOfIndependentVariables, 1>						  IndependentType;
	typedef Eigen::Matrix<Precision, Dimension::SizeOfNoiseVector, 1>									  NoiseType;

private:
	Parameters					_params;
	BoundaryCondition			_boundary;
	StochasticMatrixType		_DiffusionMatrix;


private:
	StochasticMatrixType buildDiffusionMatrix(const prec& coef)
	{
		StochasticMatrixType tmp{ StochasticMatrixType::Identity()*coef };
		return tmp;
	};
	
public:

	explicit BrownianMotion(const Parameters& params) :
		_DiffusionMatrix(buildDiffusionMatrix(params.GetDiffusionCoefficient())),
		_boundary(params), _params(params)
	{

	}

	StochasticMatrixType getStochasticMatrix(const DependentType& yi) const
	{
		return _DiffusionMatrix;
	};

	DeterministicType getDrift(const DependentType& yi) const
	{
		return DependentType::Zero();
	};

	DeterministicType getDeterministicVector(const DependentType& yi, const IndependentType& xi) const
	{
		return DependentType::Zero();
	};

	__forceinline void finishCalculations(DependentType& yi) const
	{
		_boundary.checkBoundary(yi);
	};

	inline static const std::vector<std::string> getHeader()
	{
		return std::vector<std::string> { "X", "Y", "Z" };
	};

};
