///-------------------------------------------------------------------------------------------------
// file:	Force.h
//
// summary:	Declares the force class
///-------------------------------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include "GeneralField.h"

///-------------------------------------------------------------------------------------------------
/// <summary>	A base force class CRTP static interface. </summary>
///
/// <remarks>	Neumann, 07.07.2016. </remarks>
///
/// <typeparam name="prec">		   	Type of the prec. </typeparam>
/// <typeparam name="DerivedForce">	Type of the derived force. </typeparam>
///-------------------------------------------------------------------------------------------------


//NOT NEEDED Force are Described by ForceField -> Defined by GeneralField;
//template <typename prec, typename DerivedForce>
//class BaseForce
//{
//public:
//	typedef Eigen::Matrix<prec, 3, 1>	ForceVector;
//	typedef Eigen::Matrix<prec, 3, 1>	PositionVector;
//private:
//	__forceinline DerivedForce& force() // Handy helper to remove the static_cast in the other functions
//	{
//		return *static_cast<DerivedForce*>(this);
//	};
//
//public:
//
//	///-------------------------------------------------------------------------------------------------
//	/// <summary>	Gets force vector for the given psotion. </summary>
//	///
//	/// <remarks>	Neumann, 07.07.2016. </remarks>
//	///
//	/// <param name="pos">	The position. </param>
//	///
//	/// <returns>	The force vector. </returns>
//	///-------------------------------------------------------------------------------------------------
//
//	__forceinline ForceVector getForceVector(const PositionVector& pos) noexcept const
//	{
//		return force().getForceVector(pos);
//	}
//
//};

template <typename prec>
class NoForce : public GeneralField<prec, NoForce<prec>,3>
{
	typedef Eigen::Matrix<prec, 3, 1> FieldVector;

	__forceinline FieldVector getField(const prec time)
	{
		return FieldVector::Zero();
	};
};