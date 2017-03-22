///---------------------------------------------------------------------------------------------------
// file:	SpatialInformation.h
//
// summary: 	Declares the spatial information class
//
//			Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 05.06.2016

#ifndef INC_SpatialInformation_H
#define INC_SpatialInformation_H
///---------------------------------------------------------------------------------------------------
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <sstream>
#include <iomanip>
#include <string>

#include <Eigen/Core>

#include "..\Archive\NamedValue.h"

///-------------------------------------------------------------------------------------------------
/// <signature>	Properties. </signature>
///
/// <summary>	Namespace for Property Classes. </summary>
///-------------------------------------------------------------------------------------------------
namespace Properties
{
	///-------------------------------------------------------------------------------------------------
	/// <summary>	Information about Spatial Information such as Position and Orientation. </summary>
	///
	/// <remarks>	Alexander Neumann, 05.06.2016. </remarks>
	///
	/// <seealso cref="T:IConfigFileAll{SpatialInformation{prec}}"/>
	/// <seealso cref="T:IMATLABFileWriteable"/>
	///-------------------------------------------------------------------------------------------------
	template<typename prec>
	class SpatialInformation
	{
	private:

		typedef SpatialInformation<prec>		ThisClass;
		
		typedef Eigen::Matrix<prec, 3, 1>			Vec3D;
		typedef Eigen::Matrix<prec, 3, 3>			Mat3D;

	private:

		Vec3D			Position{ Vec3D::Zero() };				//!< Position of the Particle in Space
		Vec3D			EulerAngles{ Vec3D::Zero() };			//!< Rotation of the Particle in Respect to a fixed reference frame in rad;

	public:
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Spatial information. </summary>
		///
		/// <typeparam name="prec">	Type of floating point precission. </typeparam>
		/// <param name="Position">   	The position. </param>
		/// <param name="EulerAngles">	The orientation in euler angles. </param>
		///-------------------------------------------------------------------------------------------------
		constexpr explicit SpatialInformation<prec>(const Vec3D& Position, const Vec3D& EulerAngles) : Position(Position), EulerAngles(EulerAngles) {};

		/// <summary>	Calculates the Rotation Matrix from the given Euler Angles. </summary>
		constexpr inline decltype(auto) getEulerRotationMatrix() const noexcept
		{
			Mat3D tmp;
			const auto &a = EulerAngles[0]; //!< Alpha
			const auto &b = EulerAngles[1];	//!< Beta
			const auto &g = EulerAngles[2]; //!< Gamma
			tmp <<	  cos(a)*cos(g) - sin(a)*cos(b)*sin(g),	 sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
					- cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
					  sin(a)*sin(b), - cos(a)*sin(b) , cos(b);
			
			return tmp;
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Rotates a Vector into the local reference frame  </summary>
		///
		/// <param name="Vec">	The Vector to Rotate </param>
		/// 
		/// <returns>	The rotated Vector. </returns>
		///-------------------------------------------------------------------------------------------------
		constexpr inline decltype(auto) RotateVectorIntoLocalSpace(const Vec3D& Vec) const noexcept
		{
			return (getEulerRotationMatrix()*Vec);
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Calculates the given Vector seen from the local space </summary>
		///
		/// <param name="Vec">	The Vector to transform </param>
		/// 
		/// <returns>	The transformed Vector. </returns>
		///-------------------------------------------------------------------------------------------------
		constexpr inline decltype(auto) MoveVectorIntoLocalSpace(const Vec3D& Vec) const noexcept
		{
			return (Vec - Position);
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets section name. </summary>
		///
		/// <returns>	The section name. </returns>
		///-------------------------------------------------------------------------------------------------
		static inline std::string getSectionName() { return std::string{ "Spatial_Properties" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue("Position", Position));
			ar(Archives::createNamedValue("Direction", EulerAngles));
		}
	};
}

#endif	// INC_SpatialInformation_H
// end of SpatialInformation.h
///---------------------------------------------------------------------------------------------------
