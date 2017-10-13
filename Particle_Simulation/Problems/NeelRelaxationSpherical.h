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


//Eigen::AngleAxis<Precision> yAxisRotation{};
//Eigen::Transform<Precision, 3, 0> T{ Eigen::AngleAxis<Precision>{pi / 2,Vec3D(0.,1.,0.)} };


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
		
		using DeterministicType				= typename Traits::DeterministicType;
		using DependentType					= typename Traits::DependentType;
		using IndependentType				= typename Traits::IndependentType;
		using NoiseType						= typename Traits::NoiseType;
		using StochasticMatrixType			= typename Traits::StochasticMatrixType;
		using JacobiMatrixType				= typename Traits::JacobiMatrixType;
		using CoordinateTransformationType	= typename Traits::CoordinateTransformationType;
		using OutputType					= typename Traits::OutputType;
		
		template<typename T>
		using BaseMatrixType = typename Traits::template BaseMatrixType<T>;

		using Matrix_3x3				= typename Traits::Matrix_3x3;

	private: // Important: Have often used Parameters at the top of the class defintion!

		 //Particle Parameters
		Helpers::NeelParams<Precision>	mParams;

		//Helper Matrix
		IndependentType mEasyAxis;
		
		const struct 
		{
			const bool			  RotateCoordinateSystem = false;
			const Precision		  MinAngleBeforeRotation = std::numeric_limits<Precision>::epsilon();
			const Precision		  MaxAngleBeforeRotation = math::constants::pi<Precision> - MinAngleBeforeRotation;
		} mCoordSystemRotation;
		

	protected:
		//Cache Values
		bool				  isRotated = false;
		Precision			  one_div_sin_t	= 0;

		IndependentType e_cart{ IndependentType::Zero() };
		IndependentType e_theta{ IndependentType::Zero() };
		IndependentType e_phi{ IndependentType::Zero() };

		StochasticMatrixType  ProjectionMatrix{ StochasticMatrixType::Zero() };
	
		CoordinateTransformationType Jacobi_er;
		CoordinateTransformationType Jacobi_theta;
		CoordinateTransformationType Jacobi_phi;

	private:
		const Anisotropy				mAnisotropy;
		const ProblemSettings			mProblemSettings;

	public:
		//TODO: Move those out of this class!
		const UsedProperties		_ParParams;
		const InitSettings          _Init;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		explicit NeelRelaxationSpherical(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
			GeneralSDEProblem<NeelRelaxationSpherical<precision, aniso>>(NeelSphericalDimensionVar),
			mParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
			mEasyAxis(calcEasyAxis(Init)),
			mCoordSystemRotation({ ProbSettings.mUseCoordinateTransformation, ProbSettings.mMinAngleBeforeTransformation, math::constants::pi<Precision> -ProbSettings.mMinAngleBeforeTransformation }),
			mAnisotropy(Properties.getMagneticProperties()),
			mProblemSettings(ProbSettings), _ParParams(Properties), _Init(Init)
		{};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Prepares the calculations. IMPORTANT: Must be called before any call to other 
		/// 			functions</summary>
		///
		/// <param name="yi">	[in,out] The current state. </param>
		///-------------------------------------------------------------------------------------------------
		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareCalculations(BaseMatrixType<Derived>& yi)
		{
			staticVectorChecks(yi, DependentType{});
			// Only calculate these values once! Calls to sin and cos can be / are expensive!
			//const auto& theta = yi(0);//yi.template head<1>();
			//const auto& phi = yi(1);//yi.template tail<1>();
			//IndependentType e_theta{ IndependentType::Zero() };
			//IndependentType e_phi{ IndependentType::Zero() };

			if (needsCoordRotation(yi))
			{
				//std::system("pause");
				//std::cout << "Coordinates Rotated!\n";
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

			if (!isRotated) //Not rotated case
			{
				e_cart(0) = sin_t*cos_p;
				e_cart(1) = sin_t*sin_p;
				e_cart(2) = cos_t;

				e_theta(0) = cos_t*cos_p;
				e_theta(1) = cos_t*sin_p;
				e_theta(2) = -sin_t;
				//e_theta.normalize();

				e_phi(0) = -sin_p;
				e_phi(1) = cos_p;
				e_phi(2) = 0.0;
				//e_phi.normalize();
			}
			else // rotated case
			{
				//We simply apply the Rotation to the unit vectors and thus swap our helper matrix.
				//This works du to the following: H'.e_theta = (Ry.H)'.e_theta2 = H'.Ry'.e_theta2  = H'.(Ry'.e_theta2)
				//This means e_theta = Ry'.e_theta with Ry' = Ry^-1; Ry is 90° rotation matrix around y-axis
				//We also dont care if it is H'.e_theta or e_theta'.H since both are vectors (dot product). The results remains the same.

				e_cart(0) = -cos_t;
				e_cart(1) = sin_t*sin_p;
				e_cart(2) = sin_t*cos_p;

				e_theta(0) = sin_t;
				e_theta(1) = cos_t*sin_p;
				e_theta(2) = cos_t*cos_p;
				//e_theta.normalize();

				e_phi(0) = 0.0;
				e_phi(1) = cos_p;
				e_phi(2) = -sin_p;
				//e_phi.normalize();
			}

			//if (isRotated)
			//{
			//	std::cout << "e_cart:\n" << e_cart.transpose() << "\n";
			//	std::cout << "e_theta:\n" << e_theta.transpose() << "\n";
			//	std::cout << "e_phi:\n" << e_phi.transpose() << "\n";
			//}

			ProjectionMatrix.template block<1, 3>(0, 0).noalias() = -mParams.NeelFactor1*e_phi + mParams.NeelFactor2*e_theta;

			one_div_sin_t = 1.0 / sin_t;

			if (std::isinf(one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			{
				//Branch prediction should ignore this branch if the coordiante system is rotated
				ProjectionMatrix.template block<1, 3>(1, 0).noalias() = IndependentType::Zero();
			}
			else
			{
				//TODO: Recheck sign of jacobis!
				//ProjectionMatrix.template block<1, 3>(1, 0).noalias() = -one_div_sin_t* (mParams.NeelFactor1*e_theta + mParams.NeelFactor2*e_phi);
				ProjectionMatrix.template block<1, 3>(1, 0).noalias() = one_div_sin_t* (mParams.NeelFactor1*e_theta + mParams.NeelFactor2*e_phi);
			}

		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const BaseMatrixType<Derived>& yi) const
		{		
			staticVectorChecks(yi, DependentType{});
			return ProjectionMatrix*mParams.NoisePrefactor;
		};
		
		template<typename Derived>
		BASIC_ALWAYS_INLINE DeterministicType getDrift(const  BaseMatrixType<Derived>& yi) const
		{
			staticVectorChecks(yi, DependentType{});

			//NOTE: Drift does not depend wether the coordinate system is rotated or not!
			//		It is the same in both cases! Check with Mathematica!
			DependentType	  Drift{ DependentType::Zero() };

			const auto cos_t = isRotated ? -e_cart(0) : e_cart(2);
//			const auto sin_t = isRotated ? e_theta(0) : -e_theta(2);

			//one_div_sin_t = 1.0 / sin_t;

			if (std::isinf(one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			{
				Drift(0) = 0.0;
			}
			else
			{
				Drift(0) = -0.5*mParams.DriftPrefactor * cos_t * one_div_sin_t;
			}
			//std::cout << "Drift: " << Drift.transpose() << '\n';
			return Drift;
		};
		
		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const  BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi) const
		{
			staticVectorChecks(yi, DependentType{});
			staticVectorChecks(xi, IndependentType{});
			//const auto& theta = yi.template head<1>();
			//const auto& phi = yi.template tail<1>();
			const auto AnisotropyField{ mAnisotropy.getAnisotropyField(e_cart,mEasyAxis) };
			const auto Heff{ (AnisotropyField + xi) };
			
			//std::cout << "AnisotropyField: " << AnisotropyField.transpose() << '\n';
			//std::cout << "EffField: " << Heff.transpose() << '\n';
			//std::cout << "ProjectionMatrix: "<< ProjectionMatrix << '\n';

			return (ProjectionMatrix*Heff).eval();
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareJacobiCalculations(BaseMatrixType<Derived>& yi)
		{
			staticVectorChecks(yi, DependentType{});
			const auto cos_t = isRotated ? -e_cart(0) : e_cart(2);
			const auto sin_t = isRotated ? e_theta(0) : -e_theta(2);


			Jacobi_er.template block<1, 3>(0, 0) = e_theta;
			Jacobi_er.template block<1, 3>(1, 0) = sin_t*e_phi;
		
			Jacobi_theta.template block<1, 3>(0, 0) = -e_cart;
			Jacobi_theta.template block<1, 3>(1, 0) = cos_t*e_phi;

			Jacobi_phi.template block<1, 3>(0, 0) = IndependentType::Zero();
			Jacobi_phi.template block<1, 3>(1, 0) = isRotated ? IndependentType(0.0, e_phi(2), - e_phi(1)) : IndependentType(-e_phi(1), e_phi(0), 0.0);

			//This is correct!
			//if (isRotated)
			//{
			//	std::cout << "JacEr:\n " << Jacobi_er.transpose() << "\n";
			//	std::cout << "JacPhi:\n " << Jacobi_phi.transpose() << "\n";
			//	std::cout << "JacTheta:\n " << Jacobi_theta.transpose() << "\n";
			//}
		}

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiDeterministic(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi, const Precision& dt) const
		{
			//const DependentType& yi, const IndependentType& xi
			staticVectorChecks(yi, DependentType{});
			staticVectorChecks(xi, IndependentType{});
			
			//Deterministc Jacobi Matrix
			const auto HeffJacobi{ mAnisotropy.getJacobiAnisotropyField(e_cart, mEasyAxis) };
			const auto EffField{ (mAnisotropy.getAnisotropyField(e_cart, mEasyAxis) + xi) };

			//std::cout << "Heff\n" << EffField.transpose() << "\n";
			//std::cout << "HeffJacobi\n" << HeffJacobi*Jacobi_er.transpose() << "\n";

			JacobiMatrixType res{ JacobiMatrixType::Zero() };

			res.template block<1, 2>(0, 0).noalias() = (-mParams.NeelFactor1*Jacobi_phi + mParams.NeelFactor2*Jacobi_theta)*EffField;
			res.template block<1, 2>(0, 0).noalias() += ProjectionMatrix.template block<1, 3>(0, 0)*(HeffJacobi*Jacobi_er.transpose());
			
			//if (isRotated)
			//{
			//	std::cout << "part1phi:\n" << EffField.transpose()*(-mParams.NeelFactor1*Jacobi_phi).transpose() << "\n";
			//	std::cout << "part1theta:\n" << EffField.transpose()*( mParams.NeelFactor2*Jacobi_theta).transpose() << "\n";
			//	std::cout << "part1:\n" << EffField.transpose()*(-mParams.NeelFactor1*Jacobi_phi + mParams.NeelFactor2*Jacobi_theta).transpose() << "\n";
			//	std::cout << "part2:\n" << ProjectionMatrix.template block<1, 3>(0, 0)*(HeffJacobi*Jacobi_er.transpose()) << "\n";
			//}


			if (std::isinf(one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			{
				res.template block<1, 2>(1, 0) = DependentType::Zero();
			}
			else
			{
				const auto cos_t = isRotated ? -e_cart(0) : e_cart(2);
				const DependentType Jac_Sin_t(one_div_sin_t*one_div_sin_t*cos_t,0);

				res.template block<1, 2>(1, 0).noalias() = EffField.transpose()*(one_div_sin_t*(mParams.NeelFactor1*Jacobi_theta + mParams.NeelFactor2*Jacobi_phi).transpose()
					- (mParams.NeelFactor1*e_theta + mParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose());
				res.template block<1, 2>(1, 0).noalias() += (ProjectionMatrix.template block<1, 3>(1, 0)*HeffJacobi)*Jacobi_er.transpose();


				//if (isRotated)
				//{
				//	std::cout << "part1phi:\n" << EffField.transpose()*(-one_div_sin_t*(mParams.NeelFactor2*Jacobi_phi).transpose()) << "\n";
				//	std::cout << "part1theta:\n" << EffField.transpose()*(-one_div_sin_t*(mParams.NeelFactor1*Jacobi_theta)).transpose() << "\n";
				//	std::cout << "part2phi:\n" << EffField.transpose()*((mParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose()) << "\n";
				//	std::cout << "part2theta:\n" << EffField.transpose()*((mParams.NeelFactor1*e_theta)*Jac_Sin_t.transpose()) << "\n";
				//	std::cout << "part1:\n" << EffField.transpose()*(-one_div_sin_t*(mParams.NeelFactor1*Jacobi_theta + mParams.NeelFactor2*Jacobi_phi).transpose()) << "\n";
				//	std::cout << "part2:\n" << EffField.transpose()*((mParams.NeelFactor1*e_theta + mParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose()) << "\n";
				//	std::cout << "part3:\n" << (ProjectionMatrix.template block<1, 3>(1, 0)*HeffJacobi)*Jacobi_er.transpose();
				//}

			}

			//if (isRotated)
			//{
			//	//Rotation matrix is multiplied by right side not left changing signs!
			//	// -> Matrix Chain rule!
			//	res(0, 0) = -res(0, 0); 
			//	res(0, 1) = -res(0, 1);
			//	res(2, 0) = -res(2, 0);
			//	res(2, 1) = -res(2, 1);
			//}

			return res;
		}

		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiStochastic(const NoiseType& dW) const
		{
			JacobiMatrixType res;

			const auto cos_t = isRotated ? -e_cart(0) : e_cart(2);
			//const auto sin_t = isRotated ? e_theta(0) : -e_theta(2);

			res.template block<1, 2>(0, 0).noalias() = mParams.NoisePrefactor*(-mParams.NeelFactor1*Jacobi_phi + mParams.NeelFactor2*Jacobi_theta)*dW;

			if (std::isinf(one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			{
				res.template block<1, 2>(1, 0) = DependentType::Zero();
			}
			else
			{
				DependentType Jac_Sin_t(one_div_sin_t*one_div_sin_t*cos_t, 0);

				res.template block<1, 2>(1, 0).noalias() = dW.transpose()*mParams.NoisePrefactor*(one_div_sin_t*(mParams.NeelFactor1*Jacobi_theta + mParams.NeelFactor2*Jacobi_phi).transpose()
					- (mParams.NeelFactor1*e_theta + mParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose());
			}

			return res;
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Finishes the calculations. IMPORTANT: Must be called after each solver 
		/// 			iteration!</summary>
		///
		/// <param name="yi">	[in,out] The next/new iteration state </param>
		///-------------------------------------------------------------------------------------------------
		template<typename Derived>
		BASIC_ALWAYS_INLINE void finishCalculations(BaseMatrixType<Derived>& yi) const
		{
			staticVectorChecks(yi, DependentType{});

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

			//std::cout << "Next value: " << yi.transpose() << '\n';
		};
		template<typename Derived>
		BASIC_ALWAYS_INLINE void finishJacobiCalculations(BaseMatrixType<Derived>& jacobi) const
		{
			staticVectorChecks(jacobi, JacobiMatrixType{});

			//TODO: apply back rotation
			if (isRotated)
			{
				const auto m_cos_t = e_cart(0); // - cos_t
				const auto sin_t = e_theta(0);
				const auto cos_p = e_phi(1);
				const auto m_sin_p = e_phi(2);

				JacobiMatrixType JacCoordTransformation;

				const auto factor = 1.0/std::sqrt(1.0 - sin_t*sin_t*cos_p*cos_p);

				JacCoordTransformation(0, 0) = factor*m_cos_t*cos_p;
				JacCoordTransformation(0, 1) = m_sin_p;
				JacCoordTransformation(1, 0) = -factor*one_div_sin_t*m_sin_p;
				JacCoordTransformation(1, 1) = one_div_sin_t*m_cos_t*cos_p;
				// Note: one_div_sin_t is never infinity in the isRotated case if the class is used correctly 
				// Exception: MinAngleBeforeRotation >= pi/2 (Means Rotation is always applied which is not intended use!)

				jacobi = jacobi*JacCoordTransformation;
			}
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE OutputType calculateOutputResult(const BaseMatrixType<Derived>& yi) const
		{
			//TODO: Try to avoid the double calculation of sin and cos!
			const auto yisin = yi.array().sin();
			const auto yicos = yi.array().cos();

			//Precalculated Values;
			const auto& cos_t = yicos(0);
			const auto& cos_p = yicos(1);
			const auto& sin_t = yisin(0);
			const auto& sin_p = yisin(1);

			OutputType out(sin_t*cos_p, sin_t*sin_p, cos_t);

			return out;
		}
		inline auto getStart() noexcept
		{
			return getStart(_Init);
		};
		inline auto getStart(const InitSettings& init) noexcept
		{
			DependentType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<Precision> nd{ 0,1 };
			
			mEasyAxis = calcEasyAxis(init);
			assert(mEasyAxis.norm() >= (1. - 100.*std::numeric_limits<Precision>::epsilon()) || mEasyAxis.norm() <= (1. + 100. * std::numeric_limits<Precision>::epsilon()));


			if (init.getUseRandomInitialMagnetisationDir())
			{
				DependentType MagDir;
				for (unsigned int i = 0; i < 2; ++i)
					MagDir(i) = nd(rd);
				Result = MagDir;
			}
			else
			{
				const IndependentType tmp{ init.getInitialMagnetisationDirection() };
				IndependentType z_axis;
				IndependentType y_axis;
				IndependentType x_axis;
				x_axis << 1.0, 0.0, 0.0;
				y_axis << 0.0, 1.0, 0.0;
				z_axis << 0.0, 0.0, 1.0;
				Result(0) = std::acos(tmp.dot(z_axis)); //Theta
				Result(1) = std::atan2(tmp.dot(y_axis), tmp.dot(x_axis)); //Phi
			}

			finishCalculations(Result); //normalize if necessary

			//std::cout << "Easy axis direction: " << mEasyAxis.transpose() << '\n';
			//std::cout << "Start values: " << Result.transpose() << '\n';

			return Result;
		};
		inline auto getWeighting() const noexcept
		{
			OutputType scale{ OutputType::Ones() };
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
		template<typename Derived>
		BASIC_ALWAYS_INLINE bool needsCoordRotation(const BaseMatrixType<Derived>& yi) const noexcept
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
		template<typename Derived>
		BASIC_ALWAYS_INLINE DependentType Rotate2DSphericalCoordinate90DegreeAroundYAxis(const BaseMatrixType<Derived>& yi) const
		{
			//Rotation of Coordinates (theta,phi) to (theta',phi') 90° around y-axis;
			const auto& theta = yi(0);
			const auto& phi = yi(1);
			const auto sin_t = std::sin(theta);
			DependentType res;
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
		template<typename Derived>
		BASIC_ALWAYS_INLINE DependentType inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(const BaseMatrixType<Derived>& yi) const
		{
			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			const auto& theta = yi(0);
			const auto& phi = yi(1);
			const auto sin_t = std::sin(theta);

			DependentType res;
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
		BASIC_ALWAYS_INLINE IndependentType calcEasyAxis(const InitSettings& init) const
		{
			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<precision> nd{ 0,1 };

			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			if (init.getUseRandomInitialParticleOrientation())
			{
				IndependentType Orientation;
				for (typename IndependentType::Index i = 0; i < 3; ++i)
				{
					Orientation(i) = nd(rd);
				}
				Orientation.normalize();
				return Orientation;
			}
			else
			{
				IndependentType EulerAngles = init.getInitialParticleOrientation();
				IndependentType Orientation(1, 0, 0);
				Matrix_3x3 tmp;
				const auto &a = EulerAngles[0]; //!< Alpha
				const auto &b = EulerAngles[1];	//!< Beta
				const auto &g = EulerAngles[2]; //!< Gamma
				tmp << cos(a)*cos(g) - sin(a)*cos(b)*sin(g), sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
					-cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
					sin(a)*sin(b), -cos(a)*sin(b), cos(b);
				Orientation = tmp*Orientation;
				return Orientation;
			}
		};

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE void staticVectorChecks(const BaseMatrixType<Derived> &yi, const Derived2 &tester) const noexcept
		{
			using ToTest = Derived;
			using TestType = Derived2;
			static_assert(std::is_same_v<typename ToTest::Scalar, typename TestType::Scalar>, "Matrix scalar types do not agree!");
			static_assert(ToTest::RowsAtCompileTime == TestType::RowsAtCompileTime, "Number of rows do not agree!");
			static_assert(ToTest::ColsAtCompileTime == TestType::ColsAtCompileTime, "Number of cols do not agree!");
		}

	};
}

#include "Definitions/NeelRelaxationSpherical_Definitions.h"

#endif	// INC_NeelRelaxationSpherical_H
// end of Problems\NeelRelaxationSpherical.h
///---------------------------------------------------------------------------------------------------
