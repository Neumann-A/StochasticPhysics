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

#include <cmath>
#include <random>
#include <limits>

#include "math/math_constants.h"
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
		struct CoordinateSystem
		{
			IndependentType xAxis;
			IndependentType yAxis;
			IndependentType zAxis;
		};
		const CoordinateSystem ParticleAxes;

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
		//DependentType		  StateSines{ DependentType::Zero() };
		//DependentType		  StateCosines{ DependentType::Zero() };

		IndependentType MagnetisationDir{ IndependentType::Zero() };
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

		//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		explicit NeelRelaxationSpherical(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
			GeneralSDEProblem<NeelRelaxationSpherical<precision, aniso>>(NeelSphericalDimensionVar),
			mParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
			ParticleAxes(calculateParticleAxes(Init)),
			mCoordSystemRotation({ ProbSettings.mUseCoordinateTransformation, ProbSettings.mMinAngleBeforeTransformation, math::constants::pi<Precision> -ProbSettings.mMinAngleBeforeTransformation }),
			mAnisotropy(Properties.getMagneticProperties()),
			mProblemSettings(ProbSettings)
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

			//Prepare Sines and Cosines Cache
			DependentType StateSines = yi.array().sin();
			DependentType StateCosines = yi.array().cos();

			//Precalculated Values;
			const auto& cos_t = StateCosines(0);
			const auto& cos_p = StateCosines(1);
			const auto& sin_t = StateSines(0);
			const auto& sin_p = StateSines(1);

			one_div_sin_t = 1.0 / sin_t;

			if (std::isinf(one_div_sin_t)) {
				one_div_sin_t = 0.0;
			}

			if (!isRotated) //Not rotated case
			{
				MagnetisationDir(0) = sin_t*cos_p;
				MagnetisationDir(1) = sin_t*sin_p;
				MagnetisationDir(2) = cos_t;
				
				e_theta(0) = cos_t*cos_p;
				e_theta(1) = cos_t*sin_p;
				e_theta(2) = -sin_t;
				//e_theta.normalize();

				e_phi(0) = -sin_p;
				e_phi(1) = cos_p;
				e_phi(2) = 0.0;
				//e_phi.normalize();

				//Alternative Projection Matrix if omega is used
				//ProjectionMatrix(0, 0) = -sin_p;
				//ProjectionMatrix(1, 0) = -cos_p*cos_t*one_div_sin_t;
				//ProjectionMatrix(0, 1) = cos_p;
				//ProjectionMatrix(1, 1) = -sin_p*cos_t*one_div_sin_t;
				//ProjectionMatrix(0, 2) = 0.0;
				//ProjectionMatrix(1, 2) = 1.0;
				//ProjectionMatrix.template block<1, 3>(0, 0).noalias() = e_phi;
				//ProjectionMatrix.template block<1, 3>(1, 0).noalias() = -one_div_sin_t*(e_theta);

				const auto& a = mParams.NeelFactor1;
				const auto& b = mParams.NeelFactor2;
				ProjectionMatrix.template block<1, 3>(0, 0).noalias() = -a*e_phi + b*e_theta;
				ProjectionMatrix.template block<1, 3>(1, 0).noalias() = one_div_sin_t*(a*e_theta + b*e_phi);

				if (one_div_sin_t == 0.0) {
					ProjectionMatrix(1,2) = a;
				}
			}
			else // rotated case
			{
				//We simply apply the Rotation to the unit vectors and thus swap our helper matrix.
				//This works du to the following: H'.e_theta = (Ry.H)'.e_theta2 = H'.Ry'.e_theta2  = H'.(Ry'.e_theta2)
				//This means e_theta = Ry'.e_theta with Ry' = Ry^-1; Ry is 90° rotation matrix around y-axis
				//We also dont care if it is H'.e_theta or e_theta'.H since both are vectors (dot product). The results remains the same.

				MagnetisationDir(0) = -cos_t;
				MagnetisationDir(1) = sin_t*sin_p;
				MagnetisationDir(2) = sin_t*cos_p;
				
				e_theta(0) = sin_t;
				e_theta(1) = cos_t*sin_p;
				e_theta(2) = cos_t*cos_p;
				//e_theta.normalize();
				
				e_phi(0) = 0.0;
				e_phi(1) = cos_p;
				e_phi(2) = -sin_p;
				//e_phi.normalize();
				
				//Alternative Projection Matrix if omega is used
				//ProjectionMatrix(0, 0) = 0.0;
				//ProjectionMatrix(1, 0) = -1.0;
				//ProjectionMatrix(0, 1) = cos_p;
				//ProjectionMatrix(1, 1) = -sin_p*cos_t*one_div_sin_t;
				//ProjectionMatrix(0, 2) = -sin_p;
				//ProjectionMatrix(1, 2) = -cos_p*cos_t*one_div_sin_t;
				//ProjectionMatrix.template block<1, 3>(0, 0).noalias() = e_phi;
				//ProjectionMatrix.template block<1, 3>(1, 0).noalias() = -one_div_sin_t*(e_theta);

				const auto& a = mParams.NeelFactor1;
				const auto& b = mParams.NeelFactor2;
				ProjectionMatrix.template block<1, 3>(0, 0).noalias() = -a*e_phi + b*e_theta;
				ProjectionMatrix.template block<1, 3>(1, 0).noalias() = one_div_sin_t*(a*e_theta + b*e_phi);

				if (one_div_sin_t == 0.0) {
					ProjectionMatrix(1, 0) = -a;
				}
			}
		};

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi) const
		{
			staticVectorChecks(yi, DependentType{});
			staticVectorChecks(xi, IndependentType{});

			const auto& xAxis = ParticleAxes.xAxis;
			const auto& yAxis = ParticleAxes.yAxis;
			const auto& zAxis = ParticleAxes.zAxis;

			mAnisotropy.prepareField(MagnetisationDir, xAxis, yAxis, zAxis);
			const auto Heff{ (mAnisotropy.getAnisotropyField(MagnetisationDir,xAxis,yAxis,zAxis) + xi) };

			//std::cout << "AnisotropyField: " << AnisotropyField.transpose() << '\n';
			//std::cout << "EffField: " << Heff.transpose() << '\n';
			//std::cout << "ProjectionMatrix: "<< ProjectionMatrix << '\n';

			//Alternative calculation using omega -> needs different projection matrix!
			//const auto mxHeff = MagnetisationDir.cross(Heff).eval();
			//const auto& a = mParams.NeelFactor1;
			//const auto& b = mParams.NeelFactor2;
			//const auto omeganeel = -a*Heff + b*mxHeff;
			//return (ProjectionMatrix*omeganeel).eval();

			return (ProjectionMatrix*Heff).eval();
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const BaseMatrixType<Derived>& yi) const
		{		
			staticVectorChecks(yi, DependentType{});

			//Alternativ calculation using different projectionmatrix!
			////Neel Field Noise
			//const auto& a = mParams.NeelFactor1;
			//const auto& b = mParams.NeelFactor2;
			//const auto tmp2{ b*mParams.NoisePrefactor*MagnetisationDir };
			//Matrix_3x3 Mat;
			//Mat(0, 0) = 0.0;
			//Mat(1, 0) = tmp2(2);
			//Mat(2, 0) = -tmp2(1);
			//Mat(0, 1) = -tmp2(2);
			//Mat(1, 1) = 0.0;
			//Mat(2, 1) = tmp2(0);
			//Mat(0, 2) = tmp2(1);
			//Mat(1, 2) = -tmp2(0);
			//Mat(2, 2) = 0.0;

			//Mat -= (a*mParams.NoisePrefactor*Matrix_3x3::Identity());
			//StochasticMatrixType Result{ ProjectionMatrix*Mat };
			//return Result;

			return mParams.NoisePrefactor*ProjectionMatrix;
		};
		
		template<typename Derived>
		BASIC_ALWAYS_INLINE DeterministicType getDrift(const  BaseMatrixType<Derived>& yi) const
		{
			staticVectorChecks(yi, DependentType{});

			//NOTE: Drift does not depend wether the coordinate system is rotated or not!
			//		It is the same in both cases! Check with Mathematica!
			DependentType	  Drift{ DependentType::Zero() };

			const auto cos_t = isRotated ? -MagnetisationDir(0) : MagnetisationDir(2);
			//const auto cos_t = StateCosines(0);
			Drift(0) = -0.5*mParams.DriftPrefactor*cos_t*one_div_sin_t;

			return Drift;
		};
		



		//TODO
		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareJacobiCalculations(BaseMatrixType<Derived>& yi)
		{
			//throw std::runtime_error{ "Jacobi must be reworked! Implementation may not be correct!" };
			staticVectorChecks(yi, DependentType{});
			const auto cos_t = isRotated ? -MagnetisationDir(0) : MagnetisationDir(2);
			const auto sin_t = isRotated ? e_theta(0) : -e_theta(2);


			Jacobi_er.template block<1, 3>(0, 0) = e_theta;
			Jacobi_er.template block<1, 3>(1, 0) = sin_t*e_phi;
		
			Jacobi_theta.template block<1, 3>(0, 0) = -MagnetisationDir;
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
			
			const auto& xAxis = ParticleAxes.xAxis;
			const auto& yAxis = ParticleAxes.yAxis;
			const auto& zAxis = ParticleAxes.zAxis;

			//Deterministc Jacobi Matrix
			const auto HeffJacobi{ mAnisotropy.getJacobiAnisotropyField(MagnetisationDir,xAxis, yAxis, zAxis) };
			const auto EffField{ mAnisotropy.getAnisotropyField(MagnetisationDir,xAxis,yAxis,zAxis) + xi };

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
				const auto cos_t = isRotated ? -MagnetisationDir(0) : MagnetisationDir(2);
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

			const auto cos_t = isRotated ? -MagnetisationDir(0) : MagnetisationDir(2);

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

			if (isRotated)
			{
				yi = inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(yi);
			}
			else
			{
				yi(0) = std::fmod(yi(0), math::constants::two_pi<Precision>);
				yi(1) = std::fmod(yi(1), math::constants::two_pi<Precision>);
			}

			//yi = math::coordinates::Wrap2DSphericalCoordinatesInplace(yi);
			////Coordinates are wrapped to theta -> [0, pi]; phi -> [0,2pi)

			////TRY this instead of the wrapping; Could be faster!
			//if (isRotated)
			//{
			//	yi = inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(yi);
			//	//Coordinates are wrapped to theta -> [0, pi]; phi -> [-pi,pi]
			//	//NOTE: we dont mind the inconsistence in phi here since we only use theta for checks
			//	//		We could change Wrap2DSphericalCoordinatesInplace to -pi to pi for higer precessions 
			//	//		but it is neglectable and probably makes the wrapping code more complex (slower)

			//	//isRotated = false;
			//}

			//std::cout << "Next value: " << yi.transpose() << '\n';
		};
		template<typename Derived>
		BASIC_ALWAYS_INLINE void finishJacobiCalculations(BaseMatrixType<Derived>& jacobi) const
		{
			staticVectorChecks(jacobi, JacobiMatrixType{});

			if (isRotated)
			{
				const auto m_cos_t = MagnetisationDir(0); // - cos_t
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
			//NOTE: Currently this is 50ns faster than introducing another branch. 
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

		inline auto getStart(const InitSettings& init) noexcept
		{
			DependentType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<Precision> nd{ 0.0,1.0 };
			
			if (init.getUseRandomInitialMagnetisationDir())
			{
				DependentType MagDir;
				for (unsigned int i = 0; i < MagDir.size(); ++i)
					MagDir(i) = nd(rd);
				Result = MagDir;
                Result.normalize();
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

			//std::cout << "Particle z-Axis: " << ParticleAxes.zAxis.transpose() << '\n';
			//std::cout << "Start values: " << Result.transpose() << '\n';

			return Result;
		}
		static auto getWeighting(const UsedProperties &Properties) noexcept
		{
			OutputType scale{ OutputType::Ones() };
			return (scale * Properties.getMagneticProperties().getSaturationMoment()).eval();
		}

		template<typename Derived>
		static BASIC_ALWAYS_INLINE void normalize(BaseMatrixType<Derived>& yi) noexcept
		{		};

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
		static BASIC_ALWAYS_INLINE CoordinateSystem calculateParticleAxes(const InitSettings& init)
		{
			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::uniform_real_distribution<Precision> ud{ 0,1 };

			IndependentType EulerAngles;

			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			if (init.getUseRandomInitialParticleOrientation()) {
				for (typename IndependentType::Index i = 0; i < 3; ++i) {
					EulerAngles(i) = ud(rd)*math::constants::two_pi<Precision>;
				}
			}
			else {
				EulerAngles = init.getInitialParticleOrientation();
			}

			const auto Sines = EulerAngles.array().sin();
			const auto Cosines = EulerAngles.array().cos();

			//Define some easy bindings
			const auto& cphi = Cosines(0);
			const auto& ctheta = Cosines(1);
			const auto& cpsi = Cosines(2);
			const auto& sphi = Sines(0);
			const auto& stheta = Sines(1);
			const auto& spsi = Sines(2);

			//Phi and Psi products (used twice)
			const auto cphicpsi = cphi*cpsi;
			const auto sphicpsi = sphi*cpsi;
			const auto cphispsi = cphi*spsi;
			const auto sphispsi = sphi*spsi;

			IndependentType xAxis(cphicpsi - ctheta*sphispsi, -sphicpsi - ctheta*cphispsi, stheta*spsi);
			IndependentType yAxis(ctheta*sphicpsi + cphispsi, ctheta*cphicpsi - sphispsi, -stheta*cpsi);
			IndependentType zAxis(stheta*sphi, stheta*cphi, ctheta);
			return { xAxis, yAxis, zAxis };
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
