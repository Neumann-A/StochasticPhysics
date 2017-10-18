///---------------------------------------------------------------------------------------------------
// file:		Problems\BrownAndNeelRelaxationEulerSpherical.h
//
// summary: 	Declares the brown and neel relaxation euler spherical class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 17.10.2017

#ifndef INC_BrownAndNeelRelaxationEulerSpherical_H
#define INC_BrownAndNeelRelaxationEulerSpherical_H
///---------------------------------------------------------------------------------------------------

#include <cmath>
#include <random>
#include <limits>

#include "math/Coordinates.h"

#include "./SDEFramework/GeneralSDEProblem.h"
//#include "Helpers/ParameterCalculatorNeel.h"
#include "Helpers/ParameterCalculatorBrownAndNeel.h"


// General Plan:
// Take describtion of Neel Part from Neel Spherical and add Mechanical coupling
// For brown Part use euler angles 313 or 123 (if theta is to small to avoid the gimbal lock)
// see https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf for euler angle conversions


namespace Problems
{
	constexpr static struct BrownAndNeelRelaxationEulerSphericalDimension : GeneralSDEDimension<5, 3, 3> //thats pretty handy
	{ } BrownAndNeelRelaxationEulerSphericalDimensionVar; //too get the memory space (else the compiler will optimize it away)

	template<typename precision, typename aniso>
	class BrownAndNeelRelaxationEulerSpherical :
		public GeneralSDEProblem<BrownAndNeelRelaxationEulerSpherical<precision, aniso>>
	{
		using ThisClass = BrownAndNeelRelaxationEulerSpherical<precision, aniso>;
	public:
		using Traits = SDEProblem_Traits<ThisClass>;
		using Precision = precision;
		using Anisotropy = aniso;

		using Dimension = typename Traits::Dimension;
		using ProblemSettings = typename Traits::ProblemSettings;
		using UsedProperties = typename Traits::UsedProperties;
		using InitSettings = typename Traits::InitSettings;
		using NecessaryProvider = typename Traits::NecessaryProvider;
		using SimulationParameters = typename Traits::SimulationParameters;

		using DeterministicType = typename Traits::DeterministicType;
		using DependentType = typename Traits::DependentType;
		using IndependentType = typename Traits::IndependentType;
		using NoiseType = typename Traits::NoiseType;
		using StochasticMatrixType = typename Traits::StochasticMatrixType;
		using JacobiMatrixType = typename Traits::JacobiMatrixType;
		using CoordinateTransformationType = typename Traits::CoordinateTransformationType;
		using OutputType = typename Traits::OutputType;

		template<typename T>
		using BaseMatrixType = typename Traits::template BaseMatrixType<T>;

		using Matrix_3x3 = typename Traits::Matrix_3x3;
		using Matrix_2x3 = typename Traits::Matrix_2x3;

	private:
		using BrownDependentType = typename Traits::BrownDependentType;
		using NeelDependentType = typename Traits::NeelDependentType;
		// Important: Have often used Parameters at the top of the class defintion!
		 //Particle Parameters
		Helpers::NeelParams<Precision>			mNeelParams;
		Helpers::BrownRotationParams<Precision>	mBrownParams;

		//Helper Matrix
		const struct BrownCoordSetup
		{
			const bool			  RotateCoordinateSystem = false;
			const Precision		  MinAngleBeforeRotation = std::numeric_limits<Precision>::epsilon();
			const Precision		  MaxAngleBeforeRotation = math::constants::pi<Precision> -MinAngleBeforeRotation;
		} BrownCoordRotation;
		const struct NeelCoordSetup
		{
			const bool			  RotateCoordinateSystem = false;
			const Precision		  MinAngleBeforeRotation = std::numeric_limits<Precision>::epsilon();
			const Precision		  MaxAngleBeforeRotation = math::constants::pi<Precision> -MinAngleBeforeRotation;
		} NeelCoordRotation;


	protected:
		//Cache Values
		DependentType							StateSines{ DependentType::Zero() };
		DependentType							StateCosines{ DependentType::Zero() };

		IndependentType							MagnetisationDir{ IndependentType::Zero() };	//CurrentMagnetisationDirection -> e_r


		struct BrownHelpersStruct
		{
			bool		isRotated = false;
			Matrix_3x3	EulerRotationMatrix{ Matrix_3x3::Zero() };		//Euler Rotation Matrix
			Matrix_3x3	EulerProjectionMatrix{ Matrix_3x3::Zero() };	//Projection Matrix of omega onto Euler angles
		} BrownCache;
		struct NeelHelpersStruct
		{
			bool				  isRotated = false;
			Precision			  one_div_sin_t = 0;
			IndependentType		  e_theta{ IndependentType::Zero() };			//e_theta	in the unrotated system
			IndependentType		  e_phi{ IndependentType::Zero() };			//e_phi		in the unrotated system
			Matrix_2x3			  SphericalProjectionMatrix{ Matrix_2x3::Zero() };
		} NeelCache;





	private:
		const Anisotropy				mAnisotropy;
		const ProblemSettings			mProblemSettings;

	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		explicit BrownAndNeelRelaxationEulerSpherical(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
			GeneralSDEProblem<BrownAndNeelRelaxationEulerSpherical<precision, aniso>>(BrownAndNeelRelaxationEulerSphericalDimensionVar),
			mNeelParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(),Properties.getTemperature())),
			mBrownParams(Helpers::BrownianRotationCalculator<Precision>::calcBrownRotationParams(Properties.getHydrodynamicProperties(), Properties.getTemperature())),
			BrownCoordRotation({ ProbSettings.mUseEulerCoordinateTransformation, ProbSettings.mBrownMinAngleBeforeTransformation, math::constants::pi<Precision> -ProbSettings.mBrownMinAngleBeforeTransformation }),
			NeelCoordRotation({ ProbSettings.mUseSphericalCoordinateTransformation, ProbSettings.mNeelMinAngleBeforeTransformation, math::constants::pi<Precision> -ProbSettings.mNeelMinAngleBeforeTransformation }),
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

			//Check wether Brown part needs rotation
			if (needsBrownCoordRotation(yi))
			{
				auto& brownblock = yi.head<3>();
				brownblock = Euler313toEuler123(brownblock);
				BrownCache.isRotated = true;
			}
			else
			{
				BrownCache.isRotated = false;
			}

			//Check if Neel part must be rotated
			if (needsNeelCoordRotation(yi))
			{
				auto& neelblock = yi.tail<2>();
				neelblock = Rotate2DSphericalCoordinate90DegreeAroundYAxis(neelblock);
				NeelCache.isRotated = true;
			}
			else
			{
				NeelCache.isRotated = false;
			}

			//Prepare Sines and Cosines Cache
			StateSines = yi.array().sin();
			StateCosines = yi.array().cos();

			//Prepare Brown related cache
			if (!BrownCache.isRotated)
			{
				//Define some easy bindings
				const auto& cphi	= StateCosines(0);
				const auto& ctheta	= StateCosines(1);
				const auto& cpsi	= StateCosines(2);
				const auto& sphi	= StateSines(0);
				const auto& stheta	= StateSines(1);
				const auto& spsi	= StateSines(2);

				//Phi and Psi products (used twice)
				const auto cphicpsi = cphi*cpsi;
				const auto sphicpsi = sphi*cpsi;
				const auto cphispsi = cphi*spsi;
				const auto sphispsi = sphi*spsi;

				//theta and phi products (used twice)
				const auto cthetacphi = ctheta*cphi;
				const auto cthetasphi = ctheta*sphi;
				//const auto sthetacphi = stheta*cphi;
				//const auto sthetasphi = stheta*sphi;

				//R313 Rotationmatrix
				BrownCache.EulerRotationMatrix(0, 0) =  cphicpsi - ctheta*sphispsi;
				BrownCache.EulerRotationMatrix(1, 0) = -sphicpsi - ctheta*cphispsi;
				BrownCache.EulerRotationMatrix(2, 0) =  stheta*spsi;
				BrownCache.EulerRotationMatrix(0, 1) =  ctheta*sphicpsi + cphispsi;
				BrownCache.EulerRotationMatrix(1, 1) =  ctheta*cphicpsi - sphispsi;
				BrownCache.EulerRotationMatrix(2, 1) = -stheta*cpsi;
				BrownCache.EulerRotationMatrix(0, 2) =  stheta*sphi;
				BrownCache.EulerRotationMatrix(1, 2) =  stheta*cphi;
				BrownCache.EulerRotationMatrix(2, 2) =  ctheta;

				auto csctheta = 1 / stheta;
				if (std::isinf(csctheta))
				{
					csctheta = 0.0;
				}

				//E313Strich Inverse ProjectionMatrix (Body fixed coordinate system)
				BrownCache.EulerProjectionMatrix(0, 0) = -cthetasphi*csctheta;
				BrownCache.EulerProjectionMatrix(1, 0) = cphi;
				BrownCache.EulerProjectionMatrix(2, 0) = sphi*csctheta;
				BrownCache.EulerProjectionMatrix(0, 1) = -cthetacphi*csctheta;
				BrownCache.EulerProjectionMatrix(1, 1) = -sphi;
				BrownCache.EulerProjectionMatrix(2, 1) = cphi*csctheta;
				BrownCache.EulerProjectionMatrix(0, 2) = 1;
				BrownCache.EulerProjectionMatrix(1, 2) = 0;
				BrownCache.EulerProjectionMatrix(2, 2) = 0;
			}
			else
			{
				//Define some easy bindings
				const auto& cphi = StateCosines(0);
				const auto& ctheta = StateCosines(1);
				const auto& cpsi = StateCosines(2);
				const auto& sphi = StateSines(0);
				const auto& stheta = StateSines(1);
				const auto& spsi = StateSines(2);

				//Phi and Psi products (used twice)
				const auto cphicpsi = cphi*cpsi;
				const auto sphicpsi = sphi*cpsi;
				const auto cphispsi = cphi*spsi;
				const auto sphispsi = sphi*spsi;

				//theta and phi products (used twice)
				//const auto cthetacphi = ctheta*cphi;
				//const auto cthetasphi = ctheta*sphi;
				const auto sthetacphi = stheta*cphi;
				const auto sthetasphi = stheta*sphi;
				
				//R123 Rotationmatrix 
				BrownCache.EulerRotationMatrix(0, 0) =  ctheta*cpsi; 
				BrownCache.EulerRotationMatrix(1, 0) =  stheta*sphicpsi - cphispsi;
				BrownCache.EulerRotationMatrix(2, 0) =  stheta*cphicpsi + sphispsi;
				BrownCache.EulerRotationMatrix(0, 1) =  ctheta*spsi;
				BrownCache.EulerRotationMatrix(1, 1) =  stheta*sphispsi + cphicpsi;
				BrownCache.EulerRotationMatrix(2, 1) =  stheta*cphispsi - sphicpsi;
				BrownCache.EulerRotationMatrix(0, 2) = -stheta;
				BrownCache.EulerRotationMatrix(1, 2) =  ctheta*sphi;
				BrownCache.EulerRotationMatrix(2, 2) =  ctheta*cphi;

				auto sectheta = 1 / ctheta;
				if (std::isinf(sectheta))
				{
					sectheta = 0.0;
				}

				//E123Strich Inverse ProjectionMatrix (Body fixed coordinate system)
				BrownCache.EulerProjectionMatrix(0, 0) =  1;
				BrownCache.EulerProjectionMatrix(1, 0) =  0;
				BrownCache.EulerProjectionMatrix(2, 0) =  0;
				BrownCache.EulerProjectionMatrix(0, 1) = -sthetasphi*sectheta;
				BrownCache.EulerProjectionMatrix(1, 1) =  cphi;
				BrownCache.EulerProjectionMatrix(2, 1) =  sphi*sectheta;
				BrownCache.EulerProjectionMatrix(0, 2) = -sthetacphi*sectheta;
				BrownCache.EulerProjectionMatrix(1, 2) = -sphi;
				BrownCache.EulerProjectionMatrix(2, 2) =  cphi*sectheta;
			}

			//Prepare Neel related cache			
			if (!NeelCache.isRotated) //Not rotated case
			{
				//Precalculated Values;
				const auto& cos_t = StateCosines(3);
				const auto& cos_p = StateCosines(4);
				const auto& sin_t = StateSines(3);
				const auto& sin_p = StateSines(4);

				MagnetisationDir(0) = sin_t*cos_p;
				MagnetisationDir(1) = sin_t*sin_p;
				MagnetisationDir(2) = cos_t;

				NeelCache.e_theta(0) = cos_t*cos_p;
				NeelCache.e_theta(1) = cos_t*sin_p;
				NeelCache.e_theta(2) = -sin_t;

				NeelCache.e_phi(0) = -sin_p;
				NeelCache.e_phi(1) = cos_p;
				NeelCache.e_phi(2) = 0.0;

				NeelCache.one_div_sin_t = 1.0 / sin_t;
			}
			else // rotated case
			{
				//Precalculated Values;
				const auto& cos_t = StateCosines(3);
				const auto& cos_p = StateCosines(4);
				const auto& sin_t = StateSines(3);
				const auto& sin_p = StateSines(4);

				//We simply apply the Rotation to the unit vectors and thus swap our helper matrix.
				//This works du to the following: H'.e_theta = (Ry.H)'.e_theta2 = H'.Ry'.e_theta2  = H'.(Ry'.e_theta2)
				//This means e_theta = Ry'.e_theta with Ry' = Ry^-1; Ry is 90° rotation matrix around y-axis
				//We also dont care if it is H'.e_theta or e_theta'.H since both are vectors (dot product). The results remains the same.

				MagnetisationDir(0) = -cos_t;
				MagnetisationDir(1) = sin_t*sin_p;
				MagnetisationDir(2) = sin_t*cos_p;

				// Not really e_theta and e_phi in the rotated coordinate system
				// We just make the projection so that it fits with the new coordinates
				NeelCache.e_theta(0) = sin_t;
				NeelCache.e_theta(1) = cos_t*sin_p;
				NeelCache.e_theta(2) = cos_t*cos_p;

				NeelCache.e_phi(0) = 0.0;
				NeelCache.e_phi(1) = cos_p;
				NeelCache.e_phi(2) = -sin_p;

				NeelCache.one_div_sin_t = 1.0 / sin_t;
			}

			NeelCache.SphericalProjectionMatrix.template block<1, 3>(0, 0).noalias() = -mNeelParams.NeelFactor1*NeelCache.e_phi + mNeelParams.NeelFactor2*NeelCache.e_theta;

			if (std::isinf(NeelCache.one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			{
				//Branch prediction should ignore this branch if the coordiante system is rotated
				NeelCache.SphericalProjectionMatrix.template block<1, 3>(1, 0).noalias() = IndependentType::Zero();
				if (!NeelCache.isRotated) {
					NeelCache.SphericalProjectionMatrix(1,2) = mNeelParams.NeelFactor1;
				}
				else {
					NeelCache.SphericalProjectionMatrix(1, 2) = -mNeelParams.NeelFactor1;
				}
			}
			else
			{
				NeelCache.SphericalProjectionMatrix.template block<1, 3>(1, 0).noalias() = NeelCache.one_div_sin_t* (mNeelParams.NeelFactor1*NeelCache.e_theta + mNeelParams.NeelFactor2*NeelCache.e_phi);
			}

		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const BaseMatrixType<Derived>& yi) const
		{
			staticVectorChecks(yi, DependentType{});
			return StochasticMatrixType::Zero();
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE DeterministicType getDrift(const BaseMatrixType<Derived>& yi) const
		{
			staticVectorChecks(yi, DependentType{});

			////NOTE: Drift does not depend wether the coordinate system is rotated or not!
			////		It is the same in both cases! Check with Mathematica!
			//DependentType	  Drift{ DependentType::Zero() };

			//const auto cos_t = NeelCache.isRotated ? -MagnetisationDir(0) : MagnetisationDir(2);
			////			const auto sin_t = isRotated ? e_theta(0) : -e_theta(2);

			////NeelCache.one_div_sin_t = 1.0 / sin_t;

			//if (std::isinf(NeelCache.one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			//{
			//	Drift(0) = 0.0;
			//}
			//else
			//{
			//	Drift(0) = -0.5*mNeelParams.DriftPrefactor * cos_t * NeelCache.one_div_sin_t;
			//}
			////std::cout << "Drift: " << Drift.transpose() << '\n';
			return DeterministicType::Zero();
		};

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const  BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi) const
		{
			staticVectorChecks(yi, DependentType{});
			staticVectorChecks(xi, IndependentType{});

			const auto& xAxis = BrownCache.EulerRotationMatrix.col(0);
			//const auto& yAxis = BrownCache.EulerRotationMatrix.col(1);
			//const auto& zAxis = BrownCache.EulerRotationMatrix.col(2);
			//const auto& theta = yi.template head<1>();
			//const auto& phi = yi.template tail<1>();
			//const auto AnisotropyField{ mAnisotropy.getAnisotropyField(e_cart,mEasyAxis) };
			const auto Heff{ (mAnisotropy.getAnisotropyField(MagnetisationDir,xAxis) + xi) };

			//std::cout << "AnisotropyField: " << AnisotropyField.transpose() << '\n';
			//std::cout << "EffField: " << Heff.transpose() << '\n';
			//std::cout << "NeelCache.SphericalProjectionMatrix.: "<< NeelCache.SphericalProjectionMatrix. << '\n';
			//(NeelCache.SphericalProjectionMatrix*Heff).eval()
			return DeterministicType::Zero();
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareJacobiCalculations(BaseMatrixType<Derived>& yi)
		{
			staticVectorChecks(yi, DependentType{});
			//const auto cos_t = isRotated ? -e_cart(0) : e_cart(2);
			//const auto sin_t = isRotated ? e_theta(0) : -e_theta(2);


			//Jacobi_er.template block<1, 3>(0, 0) = e_theta;
			//Jacobi_er.template block<1, 3>(1, 0) = sin_t*e_phi;

			//Jacobi_theta.template block<1, 3>(0, 0) = -e_cart;
			//Jacobi_theta.template block<1, 3>(1, 0) = cos_t*e_phi;

			//Jacobi_phi.template block<1, 3>(0, 0) = IndependentType::Zero();
			//Jacobi_phi.template block<1, 3>(1, 0) = isRotated ? IndependentType(0.0, e_phi(2), -e_phi(1)) : IndependentType(-e_phi(1), e_phi(0), 0.0);

			////This is correct!
			////if (isRotated)
			////{
			////	std::cout << "JacEr:\n " << Jacobi_er.transpose() << "\n";
			////	std::cout << "JacPhi:\n " << Jacobi_phi.transpose() << "\n";
			////	std::cout << "JacTheta:\n " << Jacobi_theta.transpose() << "\n";
			////}
		}

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiDeterministic(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi, const Precision& dt) const
		{
			//const DependentType& yi, const IndependentType& xi
			staticVectorChecks(yi, DependentType{});
			staticVectorChecks(xi, IndependentType{});

			//Deterministc Jacobi Matrix
			//const auto HeffJacobi{ mAnisotropy.getJacobiAnisotropyField(e_cart, mEasyAxis) };
			//const auto EffField{ (mAnisotropy.getAnisotropyField(e_cart, mEasyAxis) + xi) };

			////std::cout << "Heff\n" << EffField.transpose() << "\n";
			////std::cout << "HeffJacobi\n" << HeffJacobi*Jacobi_er.transpose() << "\n";

			//JacobiMatrixType res{ JacobiMatrixType::Zero() };

			//res.template block<1, 2>(0, 0).noalias() = (-mNeelParams.NeelFactor1*Jacobi_phi + mNeelParams.NeelFactor2*Jacobi_theta)*EffField;
			//res.template block<1, 2>(0, 0).noalias() += NeelCache.SphericalProjectionMatrix.template block<1, 3>(0, 0)*(HeffJacobi*Jacobi_er.transpose());

			////if (isRotated)
			////{
			////	std::cout << "part1phi:\n" << EffField.transpose()*(-mNeelParams.NeelFactor1*Jacobi_phi).transpose() << "\n";
			////	std::cout << "part1theta:\n" << EffField.transpose()*( mNeelParams.NeelFactor2*Jacobi_theta).transpose() << "\n";
			////	std::cout << "part1:\n" << EffField.transpose()*(-mNeelParams.NeelFactor1*Jacobi_phi + mNeelParams.NeelFactor2*Jacobi_theta).transpose() << "\n";
			////	std::cout << "part2:\n" << NeelCache.SphericalProjectionMatrix.template block<1, 3>(0, 0)*(HeffJacobi*Jacobi_er.transpose()) << "\n";
			////}


			//if (std::isinf(NeelCache.one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			//{
			//	res.template block<1, 2>(1, 0) = DependentType::Zero();
			//}
			//else
			//{
			//	const auto cos_t = isRotated ? -e_cart(0) : e_cart(2);
			//	const DependentType Jac_Sin_t(NeelCache.one_div_sin_t*NeelCache.one_div_sin_t*cos_t, 0);

			//	res.template block<1, 2>(1, 0).noalias() = EffField.transpose()*(NeelCache.one_div_sin_t*(mNeelParams.NeelFactor1*Jacobi_theta + mNeelParams.NeelFactor2*Jacobi_phi).transpose()
			//		- (mNeelParams.NeelFactor1*e_theta + mNeelParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose());
			//	res.template block<1, 2>(1, 0).noalias() += (NeelCache.SphericalProjectionMatrix.template block<1, 3>(1, 0)*HeffJacobi)*Jacobi_er.transpose();


			//	//if (isRotated)
			//	//{
			//	//	std::cout << "part1phi:\n" << EffField.transpose()*(-NeelCache.one_div_sin_t*(mNeelParams.NeelFactor2*Jacobi_phi).transpose()) << "\n";
			//	//	std::cout << "part1theta:\n" << EffField.transpose()*(-NeelCache.one_div_sin_t*(mNeelParams.NeelFactor1*Jacobi_theta)).transpose() << "\n";
			//	//	std::cout << "part2phi:\n" << EffField.transpose()*((mNeelParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose()) << "\n";
			//	//	std::cout << "part2theta:\n" << EffField.transpose()*((mNeelParams.NeelFactor1*e_theta)*Jac_Sin_t.transpose()) << "\n";
			//	//	std::cout << "part1:\n" << EffField.transpose()*(-NeelCache.one_div_sin_t*(mNeelParams.NeelFactor1*Jacobi_theta + mNeelParams.NeelFactor2*Jacobi_phi).transpose()) << "\n";
			//	//	std::cout << "part2:\n" << EffField.transpose()*((mNeelParams.NeelFactor1*e_theta + mNeelParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose()) << "\n";
			//	//	std::cout << "part3:\n" << (NeelCache.SphericalProjectionMatrix.template block<1, 3>(1, 0)*HeffJacobi)*Jacobi_er.transpose();
			//	//}

			//}

			////if (isRotated)
			////{
			////	//Rotation matrix is multiplied by right side not left changing signs!
			////	// -> Matrix Chain rule!
			////	res(0, 0) = -res(0, 0); 
			////	res(0, 1) = -res(0, 1);
			////	res(2, 0) = -res(2, 0);
			////	res(2, 1) = -res(2, 1);
			////}

			return JacobiMatrixType::Zero();
		}

		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiStochastic(const NoiseType& dW) const
		{
			//JacobiMatrixType res;

			//const auto cos_t = NeelCache.isRotated ? -MagnetisationDir(0) : MagnetisationDir(2);
			////const auto sin_t = isRotated ? e_theta(0) : -e_theta(2);

			//res.template block<1, 2>(0, 0).noalias() = mNeelParams.NoisePrefactor*(-mNeelParams.NeelFactor1*Jacobi_phi + mNeelParams.NeelFactor2*Jacobi_theta)*dW;

			//if (std::isinf(NeelCache.one_div_sin_t))		//Note this should only be a problem if we do not rotate the coordinate system!
			//{
			//	res.template block<1, 2>(1, 0) = DependentType::Zero();
			//}
			//else
			//{
			//	DependentType Jac_Sin_t(NeelCache.one_div_sin_t*NeelCache.one_div_sin_t*cos_t, 0);

			//	res.template block<1, 2>(1, 0).noalias() = dW.transpose()*mNeelParams.NoisePrefactor*(NeelCache.one_div_sin_t*(mNeelParams.NeelFactor1*Jacobi_theta + mNeelParams.NeelFactor2*Jacobi_phi).transpose()
			//		- (mNeelParams.NeelFactor1*e_theta + mNeelParams.NeelFactor2*e_phi)*Jac_Sin_t.transpose());
			//}

			return JacobiMatrixType::Zero();
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

			//TODO: Do wrap coordinates!
			if (BrownCache.isRotated)
			{
				auto& brownblock = yi.head<3>();
				brownblock = Euler123toEuler313(brownblock);
			}
			if (NeelCache.isRotated)
			{
				auto& neelblock = yi.tail<2>();
				neelblock = inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(neelblock);
			}
		};
		template<typename Derived>
		BASIC_ALWAYS_INLINE void finishJacobiCalculations(BaseMatrixType<Derived>& jacobi) const
		{
			//staticVectorChecks(jacobi, JacobiMatrixType{});

			//if (NeelCache.isRotated)
			//{
			//	const auto m_cos_t = e_cart(0); // - cos_t
			//	const auto sin_t = e_theta(0);
			//	const auto cos_p = e_phi(1);
			//	const auto m_sin_p = e_phi(2);

			//	JacobiMatrixType JacCoordTransformation;

			//	const auto factor = 1.0 / std::sqrt(1.0 - sin_t*sin_t*cos_p*cos_p);

			//	JacCoordTransformation(0, 0) = factor*m_cos_t*cos_p;
			//	JacCoordTransformation(0, 1) = m_sin_p;
			//	JacCoordTransformation(1, 0) = -factor*NeelCache.one_div_sin_t*m_sin_p;
			//	JacCoordTransformation(1, 1) = NeelCache.one_div_sin_t*m_cos_t*cos_p;
			//	// Note: NeelCache.one_div_sin_t is never infinity in the isRotated case if the class is used correctly 
			//	// Exception: MinAngleBeforeRotation >= pi/2 (Means Rotation is always applied which is not intended use!)

			//	jacobi = jacobi*JacCoordTransformation;
			//}
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

		inline auto getStart(const InitSettings& init) noexcept
		{
			DependentType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<Precision> nd{ 0,1 };

			if (init.getUseRandomInitialMagnetisationDir())
			{
				DependentType MagDir;
				for (unsigned int i = 0; i < MagDir.size(); ++i)
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

			//std::cout << "Easy axis direction: " << mEasyAxis.transpose() << '\n';
			//std::cout << "Start values: " << Result.transpose() << '\n';

			return Result;
		}
		static auto getWeighting(const UsedProperties &Properties) noexcept
		{
			OutputType scale{ OutputType::Ones() };
			return (scale * Properties.getMagneticProperties().getSaturationMoment()).eval();
		}


	protected:

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Checks if the Neel coordinate system needs to be rotated </summary>
		///
		/// <param name="yi">	The state to check. </param>
		///
		/// <returns>	True if it needs to be rotated, false otherwise </returns>
		///-------------------------------------------------------------------------------------------------
		template<typename Derived>
		BASIC_ALWAYS_INLINE bool needsNeelCoordRotation(const BaseMatrixType<Derived>& yi) const noexcept
		{
			if (NeelCoordRotation.RotateCoordinateSystem)
			{
				const auto neel_theta = std::abs(yi(3));
				
				//For the following ifs we assume that theta has been wrapped to at least [-2pi, 2pi]
				assert(neel_theta <= math::constants::two_pi<Precision>);

				if (neel_theta < NeelCoordRotation.MinAngleBeforeRotation) {
					return true;
				}
				else if(neel_theta > NeelCoordRotation.MaxAngleBeforeRotation) {
					return true;
				}
			}
			return false;
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Checks if the Brown coordinate system needs to be rotated </summary>
		///
		/// <param name="yi">	The state to check. </param>
		///
		/// <returns>	True if it needs to be rotated, false otherwise </returns>
		///-------------------------------------------------------------------------------------------------
		template<typename Derived>
		BASIC_ALWAYS_INLINE bool needsBrownCoordRotation(const BaseMatrixType<Derived>& yi) const noexcept
		{
			if (BrownCoordRotation.RotateCoordinateSystem)
			{
				const auto brown_theta = std::abs(yi(1));

				//For the following ifs we assume that theta has been wrapped to at least [-2pi, 2pi]
				assert(brown_theta <= math::constants::two_pi<Precision>);

				if (brown_theta < BrownCoordRotation.MinAngleBeforeRotation) {
					return true;
				}
				else if (brown_theta > BrownCoordRotation.MaxAngleBeforeRotation) {
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
		BASIC_ALWAYS_INLINE NeelDependentType Rotate2DSphericalCoordinate90DegreeAroundYAxis(const BaseMatrixType<Derived>& yi) const
		{
			//Rotation of Coordinates (theta,phi) to (theta',phi') 90° around y-axis;
			const auto& theta = yi(0);
			const auto& phi = yi(1);
			const auto sin_t = std::sin(theta);
			NeelDependentType res;
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
		BASIC_ALWAYS_INLINE NeelDependentType inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(const BaseMatrixType<Derived>& yi) const
		{
			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			const auto& theta = yi(0);
			const auto& phi = yi(1);
			const auto sin_t = std::sin(theta);

			NeelDependentType res;
			res(0) = std::acos(std::cos(phi) * sin_t);
			res(1) = std::atan2(std::sin(phi) * sin_t, -std::cos(theta));
			//NOTE: No need to check atan2 for division by zero, will return correct value!
			return res;
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE BrownDependentType Euler123toEuler313(const BaseMatrixType<Derived>& yi) const
		{
			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			BrownDependentType Sines(yi.array().sin());
			BrownDependentType Cosines(yi.array().cos());

			const auto newphi = std::atan2(-Sines(1),Sines(0)*Cosines(1));
			const auto newtheta = std::acos(Cosines(0)*Cosines(1));
			const auto newpsi = std::atan2(Sines(0)*Sines(2)+Cosines(0)*Sines(1)*Cosines(2),Sines(0)*Cosines(2)-Cosines(0)*Sines(1)*Sines(2));

			BrownDependentType res(newphi,newtheta,newpsi);

			return res;
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE BrownDependentType Euler313toEuler123(const BaseMatrixType<Derived>& yi) const
		{
			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			BrownDependentType Sines(yi.array().sin());
			BrownDependentType Cosines(yi.array().cos());

			const auto newphi = std::atan2(Cosines(0)*Sines(1), Cosines(1));
			const auto newtheta = -std::asin(Sines(0)*Sines(1));
			const auto newpsi = std::atan2(Cosines(0)*Sines(2) + Sines(0)*Cosines(1)*Cosines(2), Cosines(0)*Cosines(2)-Sines(0)*Cosines(1)*Sines(2));

			BrownDependentType res(newphi, newtheta, newpsi);

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
		static BASIC_ALWAYS_INLINE IndependentType calcEasyAxis(const InitSettings& init)
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



#include "Definitions/BrownAndNeelRelaxationEulerSpherical_Definitions.h"

#endif	// INC_BrownAndNeelRelaxationEulerSpherical_H
// end of Problems\BrownAndNeelRelaxationEulerSpherical.h
