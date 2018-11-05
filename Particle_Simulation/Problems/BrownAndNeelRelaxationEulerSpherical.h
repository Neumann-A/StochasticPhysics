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
// For brown Part use euler angles 313 or 123 (if theta is to small to avoid the gimbal lock) -> does not work; needs same rotation as Neel case
// see https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf for Euler Angles description.


namespace Problems
{
	constexpr static struct BrownAndNeelRelaxationEulerSphericalDimension : GeneralSDEDimension<5, 3, 6> //thats pretty handy
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
		const Precision							MagneticMoment{ 0 };

		//Helper Matrix (eliminate? because we also have access to the problem settings? Maybe having this is faster -> TODO: benchmark)
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
		IndependentType							MagnetisationDir{ IndependentType::Zero() };	//CurrentMagnetisationDirection -> e_r
		DependentType							StateSines{ DependentType::Zero() };
		DependentType							StateCosines{ DependentType::Zero() };
		struct BrownHelpersStruct
		{
			bool		isRotated = false;
			Precision   csctheta = 0.0;
			Matrix_3x3	EulerRotationMatrix{ Matrix_3x3::Zero() };		//Euler Rotation Matrix
			Matrix_3x3	EulerProjectionMatrix{ Matrix_3x3::Zero() };	//Projection Matrix of omega onto Euler angles
		} BrownCache;
		struct NeelHelpersStruct
		{
			bool				  isRotated = false;
			Precision			  one_div_sin_t = 0.0;
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
			MagneticMoment(Properties.getMagneticProperties().getSaturationMoment()),
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
			//	std::cout << "Problem will be rotated\n";
				auto&& brownblock = yi.template head<3>();
				brownblock = EulertoEulerRotated(brownblock);
				BrownCache.isRotated = true;
			}
			else
			{
				BrownCache.isRotated = false;
			}

			//Check if Neel part must be rotated
			if (needsNeelCoordRotation(yi))
			{
				auto&& neelblock = yi.template tail<2>();
				neelblock = Rotate2DSphericalCoordinate90DegreeAroundYAxis(neelblock);
				NeelCache.isRotated = true;
			}
			else
			{
				NeelCache.isRotated = false;
			}

			//Prepare Sines and Cosines Cache
			//Could try to get the compiler to emit sincos call!
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
				//const auto cthetacphi = ctheta*cphi;
				//const auto cthetasphi = ctheta*sphi;
				//const auto sthetacphi = stheta*cphi;
				//const auto sthetasphi = stheta*sphi;
				const auto cthetacpsi = ctheta*cpsi;
				const auto cthetaspsi = ctheta*spsi;

				// R313 Rotationmatrix transposed
				// Transposed for fast column access and we need to calculate where the body fixed x,y and z-axis are in world space.
				BrownCache.EulerRotationMatrix(0, 0) =  cphicpsi - ctheta*sphispsi;
				BrownCache.EulerRotationMatrix(0, 1) = -sphicpsi - ctheta*cphispsi;
				BrownCache.EulerRotationMatrix(0, 2) =  stheta*spsi;
				BrownCache.EulerRotationMatrix(1, 0) =  ctheta*sphicpsi + cphispsi;
				BrownCache.EulerRotationMatrix(1, 1) =  ctheta*cphicpsi - sphispsi;
				BrownCache.EulerRotationMatrix(1, 2) = -stheta*cpsi;
				BrownCache.EulerRotationMatrix(2, 0) =  stheta*sphi;
				BrownCache.EulerRotationMatrix(2, 1) =  stheta*cphi;
				BrownCache.EulerRotationMatrix(2, 2) =  ctheta;

				BrownCache.csctheta = 1 / stheta;
				if (std::isinf(BrownCache.csctheta))
				{
					BrownCache.csctheta = 0.0;
				}

				//std::cout << "Brown csctheta: " << BrownCache.csctheta <<'\n';
				//std::cout << "Brown sectheta: " << BrownCache.sectheta <<'\n';

				//E313 Inverse
				BrownCache.EulerProjectionMatrix(0, 0) = spsi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(1, 0) = cpsi;
				BrownCache.EulerProjectionMatrix(2, 0) = -cthetaspsi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(0, 1) = -cpsi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(1, 1) = spsi;
				BrownCache.EulerProjectionMatrix(2, 1) = cthetacpsi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(0, 2) = 0.0;
				BrownCache.EulerProjectionMatrix(1, 2) = 0.0;
				BrownCache.EulerProjectionMatrix(2, 2) = 1.0;
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
				//const auto sthetacphi = stheta*cphi;
				//const auto sthetasphi = stheta*sphi;
				const auto cthetacpsi = ctheta*cpsi;
				const auto cthetaspsi = ctheta*spsi;
				
				// R313 Rotationmatrix transposed rotated backwards (because omega will be in x,y,z space and not rotated x',y',z')
				// Transposed for fast column access and we need to calculate where the body fixed x,y and z-axis are in world space.
				BrownCache.EulerRotationMatrix(0, 0) = -stheta*sphi;
				BrownCache.EulerRotationMatrix(0, 1) = -stheta*cphi;
				BrownCache.EulerRotationMatrix(0, 2) = -ctheta;
				BrownCache.EulerRotationMatrix(1, 0) = ctheta*sphicpsi + cphispsi;
				BrownCache.EulerRotationMatrix(1, 1) = ctheta*cphicpsi - sphispsi;
				BrownCache.EulerRotationMatrix(1, 2) = -stheta*cpsi;
				BrownCache.EulerRotationMatrix(2, 0) = cphicpsi - ctheta*sphispsi;
				BrownCache.EulerRotationMatrix(2, 1) = -sphicpsi - ctheta*cphispsi;
				BrownCache.EulerRotationMatrix(2, 2) = stheta*spsi;


				BrownCache.csctheta = 1 / stheta;
				if (std::isinf(BrownCache.csctheta))
				{
					BrownCache.csctheta = 0.0;
				}
				
				//E313Strich Inverse ProjectionMatrix (Body fixed coordinate system) (also rotated back)
				//BrownCache.EulerProjectionMatrix(0, 0) = -1.0;
				//BrownCache.EulerProjectionMatrix(1, 0) = 0.0;
				//BrownCache.EulerProjectionMatrix(2, 0) = 0.0;
				//BrownCache.EulerProjectionMatrix(0, 1) = -cthetacphi*BrownCache.csctheta;
				//BrownCache.EulerProjectionMatrix(1, 1) = -sphi;
				//BrownCache.EulerProjectionMatrix(2, 1) = cphi*BrownCache.csctheta;
				//BrownCache.EulerProjectionMatrix(0, 2) = -cthetasphi*BrownCache.csctheta;
				//BrownCache.EulerProjectionMatrix(1, 2) = cphi;
				//BrownCache.EulerProjectionMatrix(2, 2) = sphi*BrownCache.csctheta;

				//E313 Inverse (also rotated back)
				BrownCache.EulerProjectionMatrix(0, 0) = 0.0;
				BrownCache.EulerProjectionMatrix(1, 0) = 0.0;
				BrownCache.EulerProjectionMatrix(2, 0) = -1.0;
				BrownCache.EulerProjectionMatrix(0, 1) = -cpsi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(1, 1) = spsi;
				BrownCache.EulerProjectionMatrix(2, 1) = cthetacpsi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(0, 2) = spsi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(1, 2) = cpsi;
				BrownCache.EulerProjectionMatrix(2, 2) = -cthetaspsi*BrownCache.csctheta;

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

				NeelCache.one_div_sin_t = 1.0 / sin_t;
				if (std::isinf(NeelCache.one_div_sin_t)) {
					NeelCache.one_div_sin_t = 0.0;
				};

				NeelCache.SphericalProjectionMatrix(0,0) = -sin_p;
				NeelCache.SphericalProjectionMatrix(1,0) = -cos_p*cos_t*NeelCache.one_div_sin_t;
				NeelCache.SphericalProjectionMatrix(0,1) = cos_p;
				NeelCache.SphericalProjectionMatrix(1,1) = -sin_p*cos_t*NeelCache.one_div_sin_t;
				NeelCache.SphericalProjectionMatrix(0,2) = 0.0;
				NeelCache.SphericalProjectionMatrix(1,2) = 1.0;
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

				NeelCache.one_div_sin_t = 1.0 / sin_t;

				if (std::isinf(NeelCache.one_div_sin_t)) {
					NeelCache.one_div_sin_t = 0.0;
				};

				NeelCache.SphericalProjectionMatrix(0, 0) = 0.0;
				NeelCache.SphericalProjectionMatrix(1, 0) = -1.0;
				NeelCache.SphericalProjectionMatrix(0, 1) = cos_p;
				NeelCache.SphericalProjectionMatrix(1, 1) = -sin_p*cos_t*NeelCache.one_div_sin_t;
				NeelCache.SphericalProjectionMatrix(0, 2) = -sin_p;
				NeelCache.SphericalProjectionMatrix(1, 2) = -cos_p*cos_t*NeelCache.one_div_sin_t;
			}
		};

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const  BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi) const
		{
			staticVectorChecks(yi, DependentType{});
			staticVectorChecks(xi, IndependentType{});

			const auto& BrownEuler		= yi.template head<3>();
			const auto& BrownSines		= StateSines.template head<3>();
			const auto& BrownCosines	= StateCosines.template head<3>();
			//const auto& NeelSpherical	= yi.tail<2>();
			//const auto& NeelSines		= StateSines.tail<2>();
			//const auto& NeelCosines		= StateCosines.tail<2>();

			const auto& xAxis = BrownCache.EulerRotationMatrix.col(0);
			const auto& yAxis = BrownCache.EulerRotationMatrix.col(1);
			const auto& zAxis = BrownCache.EulerRotationMatrix.col(2);

			mAnisotropy.prepareField(MagnetisationDir, xAxis, yAxis, zAxis);
			const auto Heff{ (mAnisotropy.getAnisotropyField(MagnetisationDir,xAxis,yAxis,zAxis) + xi) };
			const auto Teff{ (mAnisotropy.getEffTorque(MagnetisationDir,xAxis,yAxis,zAxis,BrownEuler,BrownSines,BrownCosines))};
						
			DeterministicType Result;
			auto&& brownres = Result.template head<3>();
			auto&& neelres  = Result.template tail<2>();
			
			const auto& c = mBrownParams.BrownPrefactor;
			
			const auto d = c*MagneticMoment;

			const auto mxHeff = MagnetisationDir.cross(Heff).eval();
			//const auto dmxHeff = (d * mxHeff).eval(); //Benchmark this!

			const auto omegabrown = c*Teff + d*mxHeff;
			brownres = BrownCache.EulerProjectionMatrix*omegabrown;

			const auto& a = mNeelParams.NeelFactor1;
			const auto& b = mNeelParams.NeelFactor2;
			const auto omeganeel = -a*Heff+ (b+d)*mxHeff + c*Teff;
			neelres = NeelCache.SphericalProjectionMatrix*omeganeel;
			
			//For test Debugging!
			//std::cout << "xAxis:\t" << xAxis.transpose() << '\n';
			//std::cout << "yAxis:\t" << yAxis.transpose() << '\n';
			//std::cout << "zAxis:\t" << zAxis.transpose() << '\n';
			//std::cout << "crosszAxis:\t" << xAxis.cross(yAxis).transpose() << '\n';
			//std::cout << "Heff: " << Heff.transpose() << '\n';
			//std::cout << "Teff: " << Teff.transpose() << '\n';
			//std::cout << "NeelCache.SphericalProjectionMatrix: "<< NeelCache.SphericalProjectionMatrix << '\n'; 
			//std::cout << "BrownCache.EulerProjectionMatrix: " << BrownCache.EulerProjectionMatrix << '\n';
			//std::cout << "xAxis: " << xAxis.transpose() << '\n';
			//std::cout << "yAxis: " << yAxis.transpose() << '\n';
			//std::cout << "zAxis: " << zAxis.transpose() << '\n';
			//std::cout << "wNeelOnly " << ((-a*Heff).eval() + (b*mxHeff).eval()).transpose() << '\n';
			//std::cout << "c*Teff " << (c*Teff).transpose() << '\n';
			//std::cout << "d*mxHeff " << (d*mxHeff).transpose() << '\n';
			//std::cout << "MagnetisationDir: " << MagnetisationDir.transpose() << '\n';
			//std::cout << "mxHeff: " << mxHeff.transpose() << '\n';
			//std::cout << "wBrown: " << omegabrown.transpose() << '\n';
			//std::cout << "wNeel: " << omeganeel.transpose() << '\n';
			//std::cout << "a, b, c, d\t" << a << ",\t" << b << ",\t" << c << ",\t" << d << '\n';
			
			return Result;
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const BaseMatrixType<Derived>& yi) const
		{
			staticVectorChecks(yi, DependentType{});
			StochasticMatrixType Result;
			auto&& BrownTorqueNoise	= Result.template block<3, 3>(0, 0);
			auto&& BrownFieldeNoise	= Result.template block<3, 3>(0, 3);
			auto&& NeelTorqueNoise	= Result.template block<2, 3>(3, 0);
			auto&& NeelFieldNoise	= Result.template block<2, 3>(3, 3);

			//Brown Torque Noise
			//Brown_F_Noise = c*Drift
			BrownTorqueNoise.noalias() = mBrownParams.Brown_F_Noise*BrownCache.EulerProjectionMatrix;

			//std::cout << "BrownProjection:\n" << BrownCache.EulerProjectionMatrix << '\n';
			//std::cout << "Brown_F_Noise: " << mBrownParams.Brown_F_Noise << '\n';
			const auto& c = mBrownParams.BrownPrefactor;
			const auto d = c*MagneticMoment; //TODO: Make mixed parameter

			{
				//Brown Field Noise
				const auto tmp{ d*mNeelParams.NoisePrefactor*MagnetisationDir };
				BrownFieldeNoise(0, 0) = 0.0;
				BrownFieldeNoise(1, 0) = tmp(2);
				BrownFieldeNoise(2, 0) = -tmp(1);
				BrownFieldeNoise(0, 1) = -tmp(2);
				BrownFieldeNoise(1, 1) = 0.0;
				BrownFieldeNoise(2, 1) = tmp(0);
				BrownFieldeNoise(0, 2) = tmp(1);
				BrownFieldeNoise(1, 2) = -tmp(0);
				BrownFieldeNoise(2, 2) = 0.0;
				BrownFieldeNoise.applyOnTheLeft(BrownCache.EulerProjectionMatrix);
				//BrownFieldeNoise = BrownCache.EulerProjectionMatrix*BrownFieldeNoise;
			}
			//Neel Torque Noise
			// NeelCache.SphericalProjectionMatrix
			// mBrownParams.Brown_F_Noise
			NeelTorqueNoise.noalias() = NeelCache.SphericalProjectionMatrix*(mBrownParams.Brown_F_Noise*Matrix_3x3::Identity());

			{
				//Neel Field Noise
				const auto& a = mNeelParams.NeelFactor1;
				const auto& b = mNeelParams.NeelFactor2;

				const auto tmp2{ (b + d)*mNeelParams.NoisePrefactor*MagnetisationDir };
				Matrix_3x3 Mat;
				Mat(0, 0) = 0.0;
				Mat(1, 0) = tmp2(2);
				Mat(2, 0) = -tmp2(1);
				Mat(0, 1) = -tmp2(2);
				Mat(1, 1) = 0.0;
				Mat(2, 1) = tmp2(0);
				Mat(0, 2) = tmp2(1);
				Mat(1, 2) = -tmp2(0);
				Mat(2, 2) = 0.0;

				//std::cout << "magdir" << MagnetisationDir.transpose() << '\n';
				//std::cout << "(b+d)*noise " << (b + d)*mNeelParams.NoisePrefactor << '\n';
				//std::cout << "(b+d) " << (b + d) << '\n';
				//std::cout << "noise " << mNeelParams.NoisePrefactor << '\n';
				//std::cout << "b " << b << '\n';
				//std::cout << "d " << d << '\n';
				//std::cout << "Mat:\n" << Mat << '\n';

				Mat -= (a*mNeelParams.NoisePrefactor*Matrix_3x3::Identity());
				NeelFieldNoise.noalias() = NeelCache.SphericalProjectionMatrix*Mat;

				//std::cout << "Mat:\n" << Mat << '\n';
			}
			return Result;
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE DeterministicType getDrift(const BaseMatrixType<Derived>& yi) const
		{
			staticVectorChecks(yi, DependentType{});
			DeterministicType Result;
			auto&& BrownDrift = Result.template head<3>();
			auto&& NeelDrift = Result.template tail<2>();

			//Shortcuts (should be optimized away)
			const auto& a = mNeelParams.NeelFactor1;
			const auto& b = mNeelParams.NeelFactor2;
			const auto& c = mBrownParams.BrownPrefactor;
			const auto d = c*MagneticMoment; //TODO: Make mixed parameter
			const auto& DN = mNeelParams.NoisePrefactor;
			const auto& DB = mBrownParams.BrownDiffusion;
			
			//Brown angles
			//const auto& cphi = StateCosines(0);
			const auto& ctheta = StateCosines(1);
			const auto& cpsi = StateCosines(2);
			//const auto& sphi = StateSines(0);
			//const auto& stheta = StateSines(1);
			const auto& spsi = StateSines(2);

			//Neel angles
			const auto& cos_t = StateCosines(3);
			const auto& cos_p = StateCosines(4);
			const auto& sin_t = StateSines(3);
			const auto& sin_p = StateSines(4);
			
			const auto& csctheta = BrownCache.csctheta;

			const auto DN_2 = DN*DN;
			const auto DB_2 = DB*DB;
			//const auto c_2 = c*c;
			const auto c_2_half = 0.5*c*c;
			//const auto d_2 = d*d;
			const auto d_2_fourth = 0.25*d*d;
			const auto a_d = a*d;

			const auto bpd = b + d;

			//The drift on the Neel part does not depend on the coordinate system
			if ((!BrownCache.isRotated && !NeelCache.isRotated) || (BrownCache.isRotated && NeelCache.isRotated))
			{
				//Helper
				const auto cotb = csctheta*ctheta;
				const auto cos_2t = cos_t*cos_t - sin_t*sin_t;
				const auto sin_2t = 2.0*cos_t*sin_t;
				const auto sin_t_2 = sin_t*sin_t;
				const auto cos_phinmpsib = cos_p*cpsi + spsi*sin_p;
				const auto sin_phinmpsib = cpsi*sin_p - cos_p*spsi;
				const auto twophineelminuspsibrown = 2.0*(yi(4) - yi(2));
				const auto cos_2phinmpsib = std::cos(twophineelminuspsibrown);
				const auto sin_2phinmpsib = std::sin(twophineelminuspsibrown);

				BrownDrift(0) = -DN_2*csctheta*sin_t*(-a*d*sin_phinmpsib+2.0*d_2_fourth*(cos_t*cos_phinmpsib+cotb*sin_t*sin_2phinmpsib));
				BrownDrift(1) = c_2_half*DB_2*cotb - DN_2*a_d*(cos_phinmpsib*sin_t)+d_2_fourth*DN_2*(0.5*(3.0+cos_2t+2.0*cos_2phinmpsib*sin_t_2)*cotb-sin_2t*sin_phinmpsib);
				BrownDrift(2) = -DN_2*(a_d*(cos_t+sin_t*cotb*sin_phinmpsib)-d_2_fourth*sin_t*(2.0*cos_t*cotb*cos_phinmpsib+(-1.0+2.0*csctheta*csctheta)*sin_t*sin_2phinmpsib));
			}
			else if(BrownCache.isRotated)
			{
				const auto sin_2t = 2.0*cos_t*sin_t;
				const auto sin_t_2 = sin_t*sin_t;
				const auto cpsi_2 = cpsi*cpsi;
				const auto spsi_2 = spsi*spsi;
				const auto c2psi = cpsi_2 - spsi_2;
				const auto s2psi = 2.0*cpsi*spsi;
				const auto sin_2p = 2.0*cos_p*sin_p;
				const auto cotb = csctheta*ctheta;
				const auto cos_t_2 = cos_t*cos_t;
				const auto sin_p_2 = sin_p*sin_p;
				const auto cos_p_2 = cos_p*cos_p;

				BrownDrift(0) = DN_2*csctheta*(a_d*(-spsi*cos_t+sin_p*cpsi*sin_t)+2.0*d_2_fourth*(-c2psi*cotb*sin_2t*sin_p+cos_p*sin_t*(cos_t*cpsi + sin_t*sin_p*spsi)+cotb*s2psi*(cos_t_2-sin_t_2*sin_p_2)));
				BrownDrift(1) = c_2_half*DB_2*cotb + DN_2*(a_d*(-cos_t*cpsi-spsi*sin_t*sin_p)+ d_2_fourth*(2.0*cpsi_2*cotb*(cos_t_2+cos_p_2*sin_t_2)+cpsi*sin_t_2*sin_2p+2.0*spsi*(-sin_t*cos_t*cos_p+cotb*sin_t_2*spsi)+cotb*sin_2t*sin_p*s2psi));
				BrownDrift(2) = DN_2*(a_d*(sin_t*cos_p-cpsi*cotb*sin_t*sin_p+cotb*cos_t*spsi)- d_2_fourth*(cos_t*cpsi+sin_t*sin_p*spsi)*(2.0*cos_p*cotb*sin_t-(3.0+c2psi)*csctheta*csctheta*(cpsi*sin_t*sin_p-cos_t*spsi)));
			}
			else //Only Neel is Rotated 
			{
				const auto sin_2t = 2.0*cos_t*sin_t;
				const auto sin_t_2 = sin_t*sin_t;
				const auto cpsi_2 = cpsi*cpsi;
				const auto spsi_2 = spsi*spsi;
				const auto c2psi = cpsi_2 - spsi_2;
				const auto s2psi = 2.0*cpsi*spsi;
				const auto sin_2p = 2.0*cos_p*sin_p;
				const auto cotb = csctheta*ctheta;
				const auto cos_t_2 = cos_t*cos_t;
				const auto sin_p_2 = sin_p*sin_p;
				const auto cos_p_2 = cos_p*cos_p;

				BrownDrift(0) = DN_2*csctheta*(a_d*(spsi*cos_t + sin_p*cpsi*sin_t) + 2.0*d_2_fourth*(c2psi*cotb*sin_2t*sin_p + cos_p*sin_t*(cos_t*cpsi - sin_t*sin_p*spsi) + cotb*s2psi*(cos_t_2 - sin_t_2*sin_p_2)));
				BrownDrift(1) = c_2_half*DB_2*cotb + DN_2*(a_d*(+cos_t*cpsi - spsi*sin_t*sin_p) + d_2_fourth*(2.0*cpsi_2*cotb*(cos_t_2 + cos_p_2*sin_t_2) - cpsi*sin_t_2*sin_2p + 2.0*spsi*(-sin_t*cos_t*cos_p + cotb*sin_t_2*spsi) - cotb*sin_2t*sin_p*s2psi));
				BrownDrift(2) = DN_2*(a_d*(-sin_t*cos_p - cpsi*cotb*sin_t*sin_p - cotb*cos_t*spsi) - d_2_fourth*(cos_t*cpsi - sin_t*sin_p*spsi)*(2.0*cos_p*cotb*sin_t + (3.0 + c2psi)*csctheta*csctheta*(cpsi*sin_t*sin_p + cos_t*spsi)));
			}

			//The Neel drift term is independent of any rotation
			NeelDrift(0) = (c_2_half*DB_2 + 0.5*(a*a + bpd*bpd)*DN_2)*NeelCache.one_div_sin_t*cos_t;
			NeelDrift(1) = 0.0;

			return Result;
		};



		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareJacobiCalculations(BaseMatrixType<Derived>& yi)
		{
			staticVectorChecks(yi, DependentType{});
			assert(false); //Not Implemented yet
		}

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiDeterministic(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi, const Precision& dt) const
		{
			//const DependentType& yi, const IndependentType& xi
			staticVectorChecks(yi, DependentType{});
			staticVectorChecks(xi, IndependentType{});
			assert(false); //not implemented yet
			return JacobiMatrixType::Zero();
		}

		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiStochastic(const NoiseType& dW) const
		{
			assert(false); //Not implemented yet
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

			//TODO: Wrap coordinates!
			if (BrownCache.isRotated)
			{
			//	std::cout << "Problem is rotated\n";
				auto&& brownblock = yi.template head<3>();
			//	std::cout << "Euler angles before: " << brownblock.transpose() << '\n';
				brownblock = EulerRotatedtoEuler(brownblock);
			//	std::cout << "Euler angles after: " << brownblock.transpose() << '\n';
			}
			else
			{
				yi(0) = std::fmod(yi(0), math::constants::two_pi<Precision>);
				yi(1) = std::fmod(yi(1), math::constants::two_pi<Precision>);
				yi(2) = std::fmod(yi(2), math::constants::two_pi<Precision>);
			}

			if (NeelCache.isRotated)
			{
				auto&& neelblock = yi.template tail<2>();
				neelblock = inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(neelblock);
			}
			else
			{
				yi(3) = std::fmod(yi(3), math::constants::two_pi<Precision>);
				yi(4) = std::fmod(yi(4), math::constants::two_pi<Precision>);
			}
		};
		template<typename Derived>
		BASIC_ALWAYS_INLINE void finishJacobiCalculations(BaseMatrixType<Derived>& jacobi) const
		{
			assert(false); //Not implemented yet
			//staticVectorChecks(jacobi, JacobiMatrixType{});
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE OutputType calculateOutputResult(const BaseMatrixType<Derived>& yi)
		{
			OutputType out;
			auto&& xAxis = out.template head<3>();
			auto&& yAxis = out.template block<3,1>(3,0);
			auto&& MagDir = out.template tail<3>();
#ifdef __llvm__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wshadow"
#endif
			//Prepare Sines and Cosines Cache (will shadow here!)
			const auto StateSines = yi.array().sin();
			const auto StateCosines = yi.array().cos();
#ifdef __llvm__
#pragma clang diagnostic pop
#endif
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

			//Reminder:
			//xAxis = BrownCache.EulerRotationMatrix.col(0);
			//yAxis = BrownCache.EulerRotationMatrix.col(1);
			//zAxis = BrownCache.EulerRotationMatrix.col(2);
			//R313 Rotationmatrix transposed
			//Transposed for fast column access (we need R313 to calculate x,y and z-axis)
			//BrownCache.EulerRotationMatrix(0, 0) = cphicpsi - ctheta * sphispsi;
			//BrownCache.EulerRotationMatrix(0, 1) = -sphicpsi - ctheta * cphispsi;
			//BrownCache.EulerRotationMatrix(0, 2) = stheta * spsi;
			//BrownCache.EulerRotationMatrix(1, 0) = ctheta * sphicpsi + cphispsi;
			//BrownCache.EulerRotationMatrix(1, 1) = ctheta * cphicpsi - sphispsi;
			//BrownCache.EulerRotationMatrix(1, 2) = -stheta * cpsi;
			//BrownCache.EulerRotationMatrix(2, 0) = stheta * sphi;
			//BrownCache.EulerRotationMatrix(2, 1) = stheta * cphi;
			//BrownCache.EulerRotationMatrix(2, 2) = ctheta;

			//World (uses R313 Transposed)
			xAxis = IndependentType(cphicpsi - ctheta*sphispsi, ctheta*sphicpsi + cphispsi, stheta*sphi);
			yAxis = IndependentType(-sphicpsi - ctheta*cphispsi, ctheta*cphicpsi - sphispsi, stheta*cphi);

			const auto& cos_t = StateCosines(3);
			const auto& cos_p = StateCosines(4);
			const auto& sin_t = StateSines(3);
			const auto& sin_p = StateSines(4);
			
			MagDir = IndependentType(sin_t*cos_p, sin_t*sin_p, cos_t);
			return out;
		}

		template<typename Derived>
		static BASIC_ALWAYS_INLINE void normalize(BaseMatrixType<Derived>& yi) noexcept
		{		};

		static inline auto getStart(const InitSettings& init) noexcept
		{
			DependentType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used five times here so it is ok
			std::uniform_real_distribution<Precision> ud{ 0,1 };

			//Rotation of Coordinates (theta',phi') to (theta,phi) -90° around rotated y'-axis;
			if (init.getUseRandomInitialParticleOrientation()) {
				for (typename IndependentType::Index i = 0; i < 3; ++i)	{
					Result(i) = ud(rd)*math::constants::two_pi<Precision>;
				}
			}
			else {
				Result.template head<3>() = init.getInitialParticleOrientation();
			}

			if (init.getUseRandomInitialMagnetisationDir())	{
				for (typename IndependentType::Index i = 3; i < 5; ++i)	{
					Result(i) = ud(rd)*math::constants::two_pi<Precision>;
				}
			}
			else {
				const IndependentType tmp{ init.getInitialMagnetisationDirection() };
				IndependentType x_axis(1.0, 0.0, 0.0);
				IndependentType y_axis(0.0, 1.0, 0.0);
				IndependentType z_axis(0.0, 0.0, 1.0);
				Result(3) = std::acos(tmp.dot(z_axis)); //Theta
				Result(4) = std::atan2(tmp.dot(y_axis), tmp.dot(x_axis)); //Phi
			}

			//TODO: Wrap the result here.

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
			if (NeelCoordRotation.RotateCoordinateSystem) {
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
		BASIC_ALWAYS_INLINE BrownDependentType EulertoEulerRotated(const BaseMatrixType<Derived>& yi) const
		{
			//Rotates the coordiante system by 90 degree around y-axis and changes the euler angles accordingly
			BrownDependentType Sines(yi.array().sin());
			BrownDependentType Cosines(yi.array().cos());

			const auto newphi = std::atan2(-Cosines(2)*Cosines(0)+Cosines(1)*Sines(0)*Sines(2), Sines(0)*Cosines(2)+Cosines(1)*Cosines(0)*Sines(2));
			const auto newtheta = std::acos(-Sines(1)*Sines(2));
			const auto newpsi = std::atan2(Cosines(1),Cosines(2)*Sines(1));

			BrownDependentType res(newphi, newtheta, newpsi);

			return res;
		};
		template<typename Derived>
		BASIC_ALWAYS_INLINE BrownDependentType EulerRotatedtoEuler(const BaseMatrixType<Derived>& yi) const
		{
			//Rotates the coordiante system back by 90 degree around y-axis and changes the euler angles accordingly
			BrownDependentType Sines(yi.array().sin());
			BrownDependentType Cosines(yi.array().cos());

			const auto newphi = std::atan2(Cosines(0)*Cosines(2)-Cosines(1)*Sines(0)*Sines(2),-Cosines(2)*Sines(0)-Cosines(1)*Cosines(0)*Sines(2));
			const auto newtheta = std::acos(Sines(1)*Sines(2));
			const auto newpsi = std::atan2(-Cosines(1),Cosines(2)*Sines(1));

			BrownDependentType res(newphi, newtheta, newpsi);

			return res;
		};
	
	private:
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
