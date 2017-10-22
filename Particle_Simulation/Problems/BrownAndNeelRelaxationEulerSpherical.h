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
			Precision   csctheta = 0;
			Precision   sectheta = 0;
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
				auto&& brownblock = yi.template head<3>();
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
				auto&& neelblock = yi.template tail<2>();
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

				BrownCache.csctheta = 1 / stheta;
				if (std::isinf(BrownCache.csctheta))
				{
					BrownCache.csctheta = 0.0;
				}
				BrownCache.sectheta = 1 / ctheta;
				if (std::isinf(BrownCache.sectheta))
				{
					BrownCache.sectheta = 0.0;
				}
				//E313Strich Inverse ProjectionMatrix (Body fixed coordinate system)
				BrownCache.EulerProjectionMatrix(0, 0) = -cthetasphi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(1, 0) = cphi;
				BrownCache.EulerProjectionMatrix(2, 0) = sphi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(0, 1) = -cthetacphi*BrownCache.csctheta;
				BrownCache.EulerProjectionMatrix(1, 1) = -sphi;
				BrownCache.EulerProjectionMatrix(2, 1) = cphi*BrownCache.csctheta;
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

				BrownCache.csctheta = 1 / stheta;
				if (std::isinf(BrownCache.csctheta))
				{
					BrownCache.csctheta = 0.0;
				}
				BrownCache.sectheta = 1 / ctheta;
				if (std::isinf(BrownCache.sectheta))
				{
					BrownCache.sectheta = 0.0;
				}

				//E123Strich Inverse ProjectionMatrix (Body fixed coordinate system)
				BrownCache.EulerProjectionMatrix(0, 0) =  1.0;
				BrownCache.EulerProjectionMatrix(1, 0) =  0.0;
				BrownCache.EulerProjectionMatrix(2, 0) =  0.0;
				BrownCache.EulerProjectionMatrix(0, 1) = -sthetasphi*BrownCache.sectheta;
				BrownCache.EulerProjectionMatrix(1, 1) =  cphi;
				BrownCache.EulerProjectionMatrix(2, 1) =  sphi*BrownCache.sectheta;
				BrownCache.EulerProjectionMatrix(0, 2) = -sthetacphi*BrownCache.sectheta;
				BrownCache.EulerProjectionMatrix(1, 2) = -sphi;
				BrownCache.EulerProjectionMatrix(2, 2) =  cphi*BrownCache.sectheta;
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

			const auto Heff{ (mAnisotropy.getAnisotropyField(MagnetisationDir,xAxis,yAxis,zAxis) + xi) };
			const auto Teff{ (mAnisotropy.getEffTorque(MagnetisationDir,xAxis,yAxis,zAxis,BrownEuler,BrownSines,BrownCosines))};
			
			//std::cout << "Heff: " << Heff.transpose() << '\n';
			//std::cout << "Teff: " << Teff.transpose() << '\n';
			//std::cout << "NeelCache.SphericalProjectionMatrix: "<< NeelCache.SphericalProjectionMatrix << '\n'; 
			//std::cout << "BrownCache.EulerProjectionMatrix: " << BrownCache.EulerProjectionMatrix << '\n';
			//std::cout << "xAxis: " << xAxis.transpose() << '\n';
			//std::cout << "yAxis: " << yAxis.transpose() << '\n';
			//std::cout << "zAxis: " << zAxis.transpose() << '\n';
			
			DeterministicType Result;
			auto&& brownres = Result.template head<3>();
			auto&& neelres  = Result.template tail<2>();
			
			const auto& c = mBrownParams.BrownPrefactor;
			
			const auto d = c*MagneticMoment;

			const auto mxHeff = MagnetisationDir.cross(Heff).eval();

			const auto omegabrown = c*Teff + d*mxHeff;
			brownres = BrownCache.EulerProjectionMatrix*omegabrown;

			const auto& a = mNeelParams.NeelFactor1;
			const auto& b = mNeelParams.NeelFactor2;
			const auto omeganeel = -a*Heff+ b*mxHeff + d*mxHeff + c*Teff;
			neelres = NeelCache.SphericalProjectionMatrix*omeganeel;
			
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
			BrownTorqueNoise = mBrownParams.Brown_F_Noise*BrownCache.EulerProjectionMatrix;

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
				BrownFieldeNoise = BrownCache.EulerProjectionMatrix*BrownFieldeNoise;
			}
			//Neel Torque Noise
			// NeelCache.SphericalProjectionMatrix
			// mBrownParams.Brown_F_Noise
			NeelTorqueNoise = NeelCache.SphericalProjectionMatrix*(mBrownParams.Brown_F_Noise*Matrix_3x3::Identity());

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
				NeelFieldNoise = NeelCache.SphericalProjectionMatrix*Mat;

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
			
			//Brown ANgles
			const auto& cphi = StateCosines(0);
			const auto& ctheta = StateCosines(1);
//			const auto& cpsi = StateCosines(2);
			const auto& sphi = StateSines(0);
			const auto& stheta = StateSines(1);
//			const auto& spsi = StateSines(2);

			//Neel Angles
			const auto& cos_t = StateCosines(3);
			const auto& cos_p = StateCosines(4);
			const auto& sin_t = StateSines(3);
			const auto& sin_p = StateSines(4);
			
			const auto& csctheta = BrownCache.csctheta;
			const auto& sectheta = BrownCache.sectheta;

			const auto DN_2 = DN*DN;
			const auto DB_2 = DB*DB;
			//const auto c_2 = c*c;
			const auto c_2_half = 0.5*c*c;
			//const auto d_2 = d*d;
			const auto d_2_fourth = 0.25*d*d;
			const auto a_d = a*d;

			const auto bpd = b + d;

			//The drift on the Neel part does not depend on the coordinate system
			if (!BrownCache.isRotated && !NeelCache.isRotated)
			{
				//Helper
				const auto cotb = csctheta*ctheta;
				const auto cos_2t = cos_t*cos_t - sin_t*sin_t;
				const auto sin_2t = 2.0*cos_t*sin_t;
				const auto sin_t_2 = sin_t*sin_t;
				const auto cos_pnppb = cos_p*cphi - sphi*sin_p;
				const auto sin_pnppb = cos_p*sphi + cphi*sin_p;
				const auto twophineelplusphibrown = 2.0*(yi(0) + yi(4));
				const auto cos_2pnppb = std::cos(twophineelplusphibrown);
				const auto sin_2pnppb = std::sin(twophineelplusphibrown);

				BrownDrift(0) = DN_2*(a_d*(-cos_t+cotb*sin_t*sin_pnppb)+ d_2_fourth*sin_t*(2.0*cos_t*cotb*cos_pnppb+(1-2*csctheta*csctheta)*sin_t*sin_2pnppb));
				BrownDrift(1) = c_2_half*DB_2*cotb-a_d*DN_2*cos_pnppb*sin_t+ d_2_fourth*DN_2*(cotb*(1.5+0.5*cos_2t+ cos_2pnppb*sin_t_2)+sin_2t*sin_pnppb);
				BrownDrift(2) = DN_2*csctheta*sin_t*(-a_d*sin_pnppb + 2.0* d_2_fourth*(cotb*sin_t*sin_2pnppb - cos_t*cos_pnppb));
			}
			else if(BrownCache.isRotated && NeelCache.isRotated)
			{
				const auto cos_2t = cos_t*cos_t - sin_t*sin_t;
				const auto sin_2t = 2.0*cos_t*sin_t;
				const auto sin_t_2 = sin_t*sin_t;
				const auto tanb = sectheta*stheta;
				const auto cos_pbmpn = cos_p*cphi + sphi*sin_p;
				const auto sin_pbmpn = cos_p*sphi - cphi*sin_p;
				const auto twophibrownminusphineel = 2.0*(yi(0) - yi(4));
				const auto sin_2pbmpn = std::sin(twophibrownminusphineel);

				BrownDrift(0) = DN_2*(a_d*(cos_t + cos_pbmpn*sin_t*tanb) + d_2_fourth*(-sin_t_2*sin_2pbmpn + 2.0*cos_t*sin_t*sin_pbmpn*tanb));
				BrownDrift(1) = c_2_half*DB_2*tanb+ DN_2*(a_d*sin_t*sin_pbmpn+ d_2_fourth*(-cos_pbmpn*sin_2t+(1+cos_2t+2*sin_t_2*sin_pbmpn*sin_pbmpn)*tanb));
				BrownDrift(2) = -DN_2*sectheta*sin_t*(a_d*cos_pbmpn + d_2_fourth*cos_t*sin_pbmpn);
			}
			else if(BrownCache.isRotated)
			{
				const auto sin_2t = 2.0*cos_t*sin_t;
				const auto sin_t_2 = sin_t*sin_t;
				const auto cphi_2 = cphi*cphi;
				const auto sphi_2 = sphi*sphi;
				const auto c2phi = cphi_2 - sphi_2;
				const auto s2phi = 2.0*cphi*sphi;
				const auto sin_2p = 2.0*cos_p*sin_p;
				const auto tanb = sectheta*stheta;
				const auto cos_t_2 = cos_t*cos_t;
				const auto sin_p_2 = sin_p*sin_p;
				const auto cos_p_2 = cos_p*cos_p;

				BrownDrift(0) = DN_2*(a_d*(-cos_p*sin_t+tanb*(cos_t*cphi+sin_t*sphi*sin_p))+ d_2_fourth*(-cos_t_2*s2phi+sin_2t*(c2phi*sin_p-cos_p*sphi*tanb)+sin_t_2*(s2phi*sin_p_2+cphi*sin_2p*tanb)));
				BrownDrift(1) = c_2_half*DB_2*tanb-a_d*DN_2*(cphi*sin_t*sin_p-sphi*cos_t)+ d_2_fourth*DN_2*(cphi*cos_p*sin_2t+sin_t_2*sphi*sin_2p+(2.0*cphi_2*sin_t_2+2.0*(cos_t_2+cos_p_2*sin_t_2)*sphi_2-sin_2t*s2phi*sin_p)*tanb);
				BrownDrift(2) = -DN_2*sectheta*(a_d*(cos_t*cphi + sphi*sin_t*sin_p) + d_2_fourth*(cphi*sin_t_2*sin_2p - sphi*cos_p*sin_2t));
			}
			else //Only Neel is Rotated 
			{
				const auto cotb = csctheta*ctheta;
				const auto sin_2t = 2.0*cos_t*sin_t;
				const auto sin_t_2 = sin_t*sin_t;
				const auto c2theta = ctheta*ctheta - stheta*stheta;
				const auto s2phi = 2.0*cphi*sphi;
				const auto cos_t_2 = cos_t*cos_t;
				const auto cphi_2 = cphi*cphi;
				const auto sphi_2 = sphi*sphi;
				const auto c2phi = cphi_2 - sphi_2;
				const auto cos_p_2 = cos_p*cos_p;

				BrownDrift(0) = -DN_2*(a_d*(cos_t*cotb*sphi+sin_t*(cos_p-cphi*cotb*sin_p))+ d_2_fourth*(cos_t*cphi+sin_t*sin_p*sphi)*(2.0*cos_p*sin_t*cotb+(3.0+c2theta)*csctheta*csctheta*(cos_t*sphi-cphi*sin_t*sin_p)));
				//TODO: Reorder operations!
				BrownDrift(1) = c_2_half*DB_2*cotb + a_d*DN_2*(cos_t*cphi + sin_t*sphi*sin_p) + d_2_fourth*DN_2*(2.0*cos_t_2 + cphi_2*cotb + 2.0*cos_p_2*cotb*sin_t_2 + cos_p*(-sin_2t*sphi + 2.0*cphi*sin_t_2*sin_p) + sin_p*(cotb*sin_2t*s2phi + 2.0*cotb*sin_t_2*sphi_2*sin_p));
				BrownDrift(2) = DN_2*csctheta*(a_d*(cos_t*sphi - sin_p*cphi*sin_t) + 2.0*d_2_fourth*(cos_t*(cphi*cos_p*sin_t + cos_t*cotb*s2phi) - sin_p*(c2phi*cotb*sin_2t + sin_t_2*(-cos_p*sphi + cotb*s2phi*sin_p))));
				//BrownDrift(1) = 0.25*(2* c_2*DB_2*cotb+d*DN_2*(4.0*a*cos_t*cphi+2*d*cos_t_2*c2phi*cotb+2.0*d*cos_t_2*cotb*sin_t_2+d*cos_p*(-sin_2t*sphi+2.0*cphi*sin_t_2*sin_p)+sin_p*(d*cotb*sin_2t*s2phi+2.0*sin_t*sphi*(2.0*a+d*cotb*sin_t*sphi*sin_p))));
				//BrownDrift(2) = 0.5*d*DN_2*csctheta*(cos_t*(d*cphi*cos_p*sin_t+2.0*a*sphi)+d*cos_t_2*cotb*s2phi+sin_p*(-2.0*a*cphi*sin_t-d*cphi_2*cotb*sin_2t+d*(cos_p*sin_t_2*sphi+cotb*(sin_2t*sphi_2-sin_t_2*s2phi*sin_p))));
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
				auto&& brownblock = yi.template head<3>();
				brownblock = Euler123toEuler313(brownblock);
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

			//Prepare Sines and Cosines Cache
			const auto StateSines = yi.array().sin();
			const auto StateCosines = yi.array().cos();

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

			//xAxis = BrownCache.EulerRotationMatrix.col(0);
			//yAxis = BrownCache.EulerRotationMatrix.col(1);
			//zAxis = BrownCache.EulerRotationMatrix.col(2);
			xAxis = IndependentType(ctheta*cpsi, stheta*sphicpsi - cphispsi, stheta*cphicpsi + sphispsi);
			yAxis = IndependentType(ctheta*spsi, stheta*sphispsi + cphicpsi, stheta*cphispsi - sphicpsi);

			const auto& cos_t = StateCosines(3);
			const auto& cos_p = StateCosines(4);
			const auto& sin_t = StateSines(3);
			const auto& sin_p = StateSines(4);


			MagDir = IndependentType(sin_t*cos_p, sin_t*sin_p, cos_t);
			return out;
		}

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
			//std::cout << "Euler313: " << yi.transpose() << '\n';
			//std::cout << "Euler123: " << res.transpose() << '\n';

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
