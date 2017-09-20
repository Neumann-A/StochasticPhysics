///---------------------------------------------------------------------------------------------------
// file:		Problems\NeelRelaxationSpherical.h
//
// summary: 	Declares the neel relaxation spherical class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 20.06.2017

#ifndef INC_NeelRelaxationSpherical_H
#define INC_NeelRelaxationSpherical_H
///---------------------------------------------------------------------------------------------------
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <random>
#include <limits>

#include "math/Coordinates.h"

#include "../SDEFramework/GeneralSDEProblem.h"
#include "Helpers/ParameterCalculatorNeel.h"

namespace Problems
{
	constexpr static struct NeelSphericalDimension : GeneralSDEDimension<2, 3, 3> //thats pretty handy
	{ } NeelSphericalDimensionVar; //too get the memory space (else the compiler will optimize it away)

	template<typename precision, typename aniso>
	class NeelRelaxationSpherical :
		public GeneralSDEProblem <NeelRelaxationSpherical<precision, aniso>>
	{
		using ThisClass = NeelRelaxationSpherical<precision, aniso>;
	public:
		using Traits					= SDEProblem_Traits<ThisClass>;
		using Precision					= precision;
		using Anisotropy				= aniso;

		using Dimension					= typename Traits::Dimension;
		using ProblemSettings			= typename Traits::ProblemSettings;
		using UsedProperties			= typename Traits::UsedProperties;
		using InitSettings				= typename Traits::InitSettings;
		using NecessaryProvider			= typename Traits::NecessaryProvider;
		using SimulationParameters		= typename Traits::SimulationParameters;

		using StochasticMatrixType		= typename Traits::StochasticMatrixType;
		using DeterministicVectorType	= typename Traits::DeterministicVectorType;
		using DependentVectorType		= typename Traits::DependentVectorType;
		using IndependentVectorType		= typename Traits::IndependentVectorType;
		using NoiseVectorType			= typename Traits::NoiseVectorType;

		using Matrix_3x3				= typename Traits::Matrix_3x3;

	private: // Important: Have often used Parameters at the top of the class defintion!

		 //Particle Parameters
		Helpers::NeelParams<Precision>	mParams;

		//Helper Matrix
		IndependentVectorType easyaxis;
		
		const struct 
		{
			const bool			  RotateCoordinateSystem = false;
			const Precision		  MinAngleBeforeRotation = std::numeric_limits<Precision>::epsilon();
			const Precision		  MaxAngleBeforeRotation = math::constants::pi<Precision> - MinAngleBeforeRotation;
		} mCoordSystemRotation;
		

	protected:
		//Cache Values
		bool				  isRotated = false;
		IndependentVectorType e_cart{ IndependentVectorType::Zero() };
		StochasticMatrixType  ProjectionMatrix{ StochasticMatrixType::Zero() };
		DependentVectorType	  DriftPreCalc{ DependentVectorType::Zero() };
	
	private:
		const Anisotropy				mAnisotropy;
		const ProblemSettings			mProblemSettings;

	public:
		//TODO: Move those out of this class!
		const UsedProperties		_ParParams;
		const InitSettings          _Init;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		explicit NeelRelaxationSpherical(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
			GeneralSDEProblem<NeelRelaxationSpherical<precision, aniso>>(NeelSphericalDimensionVar),
			mParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
			easyaxis(calcEasyAxis(Init)),
			mCoordSystemRotation({ ProbSettings.mUseCoordinateTransformation, ProbSettings.mMinAngleBeforeTransformation, math::constants::pi<Precision> -ProbSettings.mMinAngleBeforeTransformation }),
			mAnisotropy(Properties.getMagneticProperties()),
			mProblemSettings(ProbSettings), _ParParams(Properties), _Init(Init)
		{};

		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const DependentVectorType& yi) const
		{		
			return ProjectionMatrix*mParams.NoisePrefactor;
		};
		
		BASIC_ALWAYS_INLINE const DeterministicVectorType& getDrift(const DependentVectorType& yi) const
		{
			//NOTE: Drift does not depend wether the coordinate system is rotated or not!
			//		It is the same in both cases! Check with Mathematica!
			return DriftPreCalc;
		};

		BASIC_ALWAYS_INLINE DeterministicVectorType getDeterministicVector(const DependentVectorType& yi, const IndependentVectorType& xi) const
		{

			//const auto& theta = yi.template head<1>();
			//const auto& phi = yi.template tail<1>();

			const auto EffField{ (mAnisotropy.getAnisotropyField(e_cart,easyaxis) + xi) };
			
			//std::cout << EffField.transpose();
			//std::cout << ProjectionMatrix;

			return (ProjectionMatrix*EffField).eval();
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Prepares the calculations. IMPORTANT: Must be called before any call to other 
		/// 			functions</summary>
		///
		/// <param name="yi">	[in,out] The current state. </param>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE void prepareCalculations(DependentVectorType& yi) 
		{
			// Only calculate these values once! Calls to sin and cos can be / are expensive!
			//const auto& theta = yi(0);//yi.template head<1>();
			//const auto& phi = yi(1);//yi.template tail<1>();
			IndependentVectorType e_theta{ IndependentVectorType::Zero() };
			IndependentVectorType e_phi{ IndependentVectorType::Zero() };

			if (needsCoordRotation(yi))
			{
				yi = Rotate2DSphericalCoordinate90DegreeAroundYAxis(yi);
				isRotated = true;
			}
			else
			{
				isRotated = false;
			}
			
			const auto yisin = yi.array().sin();
			const auto yicos = yi.array().cos();

			//Precalculated Values;
			const auto& cos_t = yicos(0);
			const auto& cos_p = yicos(1);
			const auto& sin_t = yisin(0);
			const auto& sin_p = yisin(1);

			const auto one_div_sin_t = 1.0 / sin_t;
						
			if (!isRotated) //Not rotated case
			{
				e_cart(0) = sin_t*cos_p;
				e_cart(1) = sin_t*sin_p;
				e_cart(2) = cos_t;

				e_theta(0) = cos_t*cos_p;
				e_theta(1) = cos_t*sin_p;
				e_theta(2) = - sin_t;
				//e_theta.normalize();

				e_phi(0) = - sin_p;
				e_phi(1) = cos_p;
				e_phi(2) = 0.0;
				//e_phi.normalize();
			}
			else // rotated case
			{
				//We simply apply the Rotation to the unit vectors and thus swap our helper matrix.
				//This works du to the following: H'.e_theta = (Ry.H)'.e_theta2 = H'.Ry'.e_theta2  = H'.(Ry'.e_theta2)
				//This means e_theta = Ry'.e_theta with Ry' = Ry^-1; Ry is 90° rotation matrix around y-axis
				//We also dont care if it is H'.e_theta or e_theta'.H since both are vectors. The results remains the same.

				e_cart(0) = -cos_t;
				e_cart(1) = sin_t*sin_p;
				e_cart(2) = sin_t*cos_p;

				e_theta(0) = sin_t;
				e_theta(1) = cos_t*sin_p;
				e_theta(2) = cos_t*cos_p;
				//e_theta.normalize();

				e_phi(0) = 0.0;
				e_phi(1) = cos_p;
				e_phi(2) = - sin_p;
				//e_phi.normalize();
			}

			ProjectionMatrix.template block<1, 3>(0, 0) = - mParams.NeelFactor1*e_phi + mParams.NeelFactor2*e_theta;
			

			if (std::isinf(one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			{
				//Branch prediction should ignore this branch if the coordiante system is rotated
				ProjectionMatrix.template block<1, 3>(1, 0) = IndependentVectorType::Zero();
				DriftPreCalc(0) = 0.0;
			}
			else
			{
				DriftPreCalc(0) = 0.5*mParams.DriftPrefactor*cos_t / sin_t;
				ProjectionMatrix.template block<1, 3>(1, 0) = -one_div_sin_t* (mParams.NeelFactor1*e_theta + mParams.NeelFactor2*e_phi);
			}
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Finishes the calculations. IMPORTANT: Must be called after each solver 
		/// 			iteration!</summary>
		///
		/// <param name="yi">	[in,out] The next/new iteration state </param>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE void finishCalculations(DependentVectorType& yi) 
		{
			yi = math::coordinates::Wrap2DSphericalCoordinatesInplace(yi);
			//Coordinates are wrapped to theta -> [0, pi]; phi -> [0,2pi)

			//TRY this instead of the wrapping; Could be faster!
			if (isRotated)
			{
				yi = inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(yi);

				//Coordinates are wrapped to theta -> [0, pi]; phi -> [-pi,pi]
				//NOTE: we dont mind the inconsistence in phi here since we only use theta for checks
				//		We could change Wrap2DSphericalCoordinatesInplace to -pi to pi for higer precessions 
				//		but it is neglectable and probably makes the wrapping code more complex (slower)

				//isRotated = false;
			}
		};
			
		inline auto getStart() noexcept
		{
			return getStart(_Init);
		};
		inline auto getStart(const InitSettings& init) noexcept
		{
			DependentVectorType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<Precision> nd{ 0,1 };
			
			easyaxis = calcEasyAxis(init);
			assert(easyaxis.norm() >= (1. - 100.*std::numeric_limits<Precision>::epislon()) || easyaxis.norm() <= (1. + 100. * std::numeric_limits<Precision>::epislon()));
			
			if (init.getUseRandomInitialMagnetisationDir())
			{
				DependentVectorType MagDir;
				for (unsigned int i = 0; i < 2; ++i)
					MagDir(i) = nd(rd);
				Result = MagDir;
			}
			else
			{
				const IndependentVectorType tmp{ init.getInitialMagnetisationDirection() };
				IndependentVectorType z_axis;
				IndependentVectorType y_axis;
				IndependentVectorType x_axis;
				x_axis << 1.0, 0.0, 0.0;
				y_axis << 0.0, 1.0, 0.0;
				z_axis << 0.0, 0.0, 1.0;
				Result(0) = std::acos(tmp.dot(z_axis)); //Theta
				Result(1) = std::atan2(tmp.dot(y_axis), tmp.dot(x_axis)); //Phi
			}
			finishCalculations(Result); //normalize if necessary

			return Result;
		};
		inline auto getWeighting() const noexcept
		{
			DependentVectorType scale{ DependentVectorType::Ones() };
			return (scale * _ParParams.getMagneticProperties().getSaturationMoment()).eval();
		};


	protected:

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Checks if the coordinate system needs to be rotated </summary>
		///
		/// <param name="yi">	The spherical coordiantes to check. </param>
		///
		/// <returns>	True if it needs to be rotated, false otherwise </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE bool needsCoordRotation(const DependentVectorType& yi)
		{
			if (mCoordSystemRotation.RotateCoordinateSystem)
			{
				const auto& theta = yi(0);
				if (theta < mCoordSystemRotation.MinAngleBeforeRotation ||
					theta > mCoordSystemRotation.MaxAngleBeforeRotation)
				{
					return true;
				}
			}
			return false;
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Rotate 2D spherical coordinate 90 degree around y coordinate axis. </summary>
		///
		/// <param name="yi">	The 2d spherical coordinate to rotate. </param>
		///
		/// <returns>	The rotated 2d coordinate. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE DependentVectorType Rotate2DSphericalCoordinate90DegreeAroundYAxis(const DependentVectorType& yi) const
		{
			//Rotation of Coordinates (theta,phi) to (theta',phi') 90° around y-axis;
			const auto& theta = yi(0);
			const auto& phi = yi(1);
			const auto sin_t = std::sin(theta);
			DependentVectorType res;
			res(0) = std::acos(-std::cos(phi) * sin_t);
			res(1) = std::atan2(std::sin(phi) * sin_t, std::cos(theta));
			//NOTE: atan2 returns values from -pi to pi; we allow the negative values here since it does not influence the result!
			return res;
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Rotate 2D spherical coordinate -90 degree around y coordinate axis. </summary>
		///
		/// <param name="yi">	The 2d spherical coordinate to rotate.  </param>
		///
		/// <returns>	The rotated 2d coordinate. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE DependentVectorType inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(const DependentVectorType& yi) const
		{
			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			const auto& theta = yi(0);
			const auto& phi = yi(1);
			const auto sin_t = std::sin(theta);

			DependentVectorType res;
			res(0) = std::acos(std::cos(phi) * sin_t);
			res(1) = std::atan2(std::sin(phi) * sin_t, - std::cos(theta));
			//NOTE: No need to check atan2 for division by zero, will return correct value!
			return res;
		};

	private:
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Calculates the direction of the easy axis. </summary>
		///
		/// <param name="init">	The initilization settings. </param>
		///
		/// <returns>	The calculated easy axis direction. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE IndependentVectorType calcEasyAxis(const InitSettings& init) const
		{
			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<precision> nd{ 0,1 };

			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			if (init.getUseRandomInitialParticleOrientation())
			{
				IndependentVectorType Orientation;
				for (std::size_t i = 0; i < 3; ++i)
				{
					Orientation(i) = nd(rd);
				}
				Orientation.normalize();
				return Orientation;
			}
			else
			{
				IndependentVectorType EulerAngles = init.getInitialParticleOrientation();
				IndependentVectorType Orientation;
				Orientation << 1, 0, 0;
				Matrix_3x3 tmp;
				const auto &a = EulerAngles[0]; //!< Alpha
				const auto &b = EulerAngles[1];	//!< Beta
				const auto &g = EulerAngles[2]; //!< Gamma
				tmp << cos(a)*cos(g) - sin(a)*cos(b)*sin(g), sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
					-cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
					sin(a)*sin(b), -cos(a)*sin(b), cos(b);
				return (tmp*Orientation).eval();
			}
		};

	};
}

#include "Definitions/NeelRelaxationSpherical_Definitions.h"

#endif	// INC_NeelRelaxationSpherical_H
// end of Problems\NeelRelaxationSpherical.h
///---------------------------------------------------------------------------------------------------
