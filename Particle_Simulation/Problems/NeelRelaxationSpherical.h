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
	public:
		typedef NeelRelaxationSpherical<precision, aniso>														ThisClass;
		typedef	SDEProblem_Traits<ThisClass>																	Traits;
		typedef precision																						Precision;

		typedef typename Traits::Dimension																	    Dimension;
		typedef typename Traits::ProblemSettings															    ProblemSettings;
		typedef typename Traits::UsedProperties																	UsedProperties;
		typedef typename Traits::InitSettings																	InitSettings;
		typedef typename Traits::NecessaryProvider																NecessaryProvider;
		typedef typename Traits::SimulationParameters															SimulationParameters;

		typedef aniso																							Anisotropy;

		typedef typename Traits::StochasticMatrixType															StochasticMatrixType;
		typedef typename Traits::DeterministicVectorType														DeterministicVectorType;
		typedef typename Traits::DependentVectorType															DependentVectorType;
		typedef typename Traits::IndependentVectorType															IndependentVectorType;
		typedef typename Traits::NoiseVectorType																NoiseVectorType;

		using Matrix_3x3 = typename Traits::Matrix_3x3;

	private: // Important: Have often used Parameters at the top of the class defintion!

			 //Particle Parameters
		Helpers::NeelParams<Precision>	_Params;
		//Helper Matrix
		IndependentVectorType easyaxis;
		

		//Cache Values
		StochasticMatrixType HelperMatrix{ StochasticMatrixType::Zero() };
		DependentVectorType DriftPreCalc{ DependentVectorType::Zero() };
		IndependentVectorType yi_cart{ IndependentVectorType::Zero() };



		const Anisotropy				_Anisotropy;
		const ProblemSettings			_ProbSet;

		constexpr BASIC_ALWAYS_INLINE Precision periodicBoundaryTheta(const Precision& theta) const noexcept
		{
			constexpr const Precision width = 2.0*M_PI;
			
			return std::remainder(theta, width);
		};
		constexpr BASIC_ALWAYS_INLINE Precision periodicBoundaryPhi(const Precision& phi) const noexcept
		{
			return std::remainder(phi, M_PI);
		}

	public:
		const UsedProperties		_ParParams;
		const InitSettings          _Init;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		explicit NeelRelaxationSpherical(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
			GeneralSDEProblem<NeelRelaxationSpherical<precision, aniso>>(NeelSphericalDimensionVar),
			_Params(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
			_Anisotropy(Properties.getMagneticProperties()),
			_ProbSet(ProbSettings), _ParParams(Properties), _Init(Init)
		{};

		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const DependentVectorType& yi) const
		{		
			return HelperMatrix*_Params.NoisePrefactor;
		};
		
		BASIC_ALWAYS_INLINE const DeterministicVectorType& getDrift(const DependentVectorType& yi) const
		{
			return DriftPreCalc;
			//TODO: Calculate!
			//return (_Params.DriftPrefactor*yi).eval();
			//return (_Params.min_e_2*yi*(1 + 2 * _Params.Damping_2 - _Params.Damping_2*yi.squaredNorm())).eval();
		};

		BASIC_ALWAYS_INLINE DeterministicVectorType getDeterministicVector(const DependentVectorType& yi, const IndependentVectorType& xi) const
		{
			//const auto& theta = yi.template head<1>();
			//const auto& phi = yi.template tail<1>();

			const auto EffField{ (_Anisotropy.getAnisotropyField(yi_cart,easyaxis) + xi) };
			
			return (HelperMatrix*EffField).eval();

			//const auto EffFieldSpherical = RotMatrix*EffField; // (H_theta, H_phi)
			//const auto& H_theta = EffFieldSpherical(0);
			//const auto& H_phi = EffFieldSpherical(1);
			//const auto& min_sin_t = RotMatrix(0, 2);
			//
			////TODO (DONE): Maybe create helper Matrix to get the result more directly, should still all 
			//const auto a_theta = _Params.NeelFactor1*H_phi +_Params.NeelFactor2*H_theta;
			//const auto a_phi = 1.0 / min_sin_t *(_Params.NeelFactor1*H_theta - _Params.NeelFactor2*H_phi);

			//DependentVectorType result;
			//result(0) = a_theta;
			//result(1) = a_phi;

			//return result;
		};

		BASIC_ALWAYS_INLINE void prepareNextStep(DependentVectorType& yi)
		{
			// Only calculate these values once! Calls to sin and cos can be expensive!
			//const auto& theta = yi(0);//yi.template head<1>();
			//const auto& phi = yi(1);//yi.template tail<1>();

			const auto yisin = yi.array().sin();
			const auto yicos = yi.array().cos();

			//Precalculated Values;
			const auto& cos_t = yicos(0);
			const auto& cos_p = yicos(1);
			const auto& sin_t = yisin(0);
			const auto& sin_p = yisin(1);
			//const auto cos_t = std::cos(theta);
			//const auto cos_p = std::cos(phi);
			//const auto sin_t = std::sin(theta);
			//const auto sin_p = std::sin(phi);

			const auto cos_t_cos_p = cos_t*cos_p;
			const auto cos_t_sin_p = cos_t*sin_p;

			auto one_div_sin_t = 1.0 / sin_t;
			//if (std::isinf(one_div_sin_t))
			//{
			//	one_div_sin_t = 0.0;
			//}
			

			yi_cart(0) = sin_t*cos_p;
			yi_cart(1) = sin_t*sin_p;
			yi_cart(2) = cos_t;
			yi_cart.normalize();

			HelperMatrix(0, 0) = _Params.NeelFactor1*cos_t_cos_p - _Params.NeelFactor2*sin_p;
			HelperMatrix(0, 1) = _Params.NeelFactor1*cos_t_sin_p + _Params.NeelFactor2*cos_p;
			HelperMatrix(0, 2) = - _Params.NeelFactor1*sin_t ;

			HelperMatrix(1, 0) = one_div_sin_t*(-_Params.NeelFactor1*sin_p - _Params.NeelFactor2*cos_t_cos_p);
			HelperMatrix(1, 1) = one_div_sin_t*(_Params.NeelFactor1*cos_p - _Params.NeelFactor2*cos_t_sin_p);
			HelperMatrix(1, 2) = -_Params.NeelFactor2;

			const auto d_2 = std::pow(_Params.NeelFactor2, 2);
			const auto c_2 = std::pow(_Params.NeelFactor1, 2);
			const auto c_d = _Params.NeelFactor1 * _Params.NeelFactor2;
			const auto cos_2t = 1.0-2.0*sin_t*sin_t; //(1-2*sin(t)^2)
			const auto sin_2p = 1.0-2.0*sin_p*sin_p; //(1-2*cos(p)^2)

			const auto cos_2p = 1.0-2.0*cos_p*cos_p; //(1-2*cos(p)^2)
			//const auto cos_2t = std::cos(2 * theta); //(1-2*sin(t)^2)
			//const auto sin_2p = std::sin(2 * phi); //(1-2*cos(p)^2)
			//const auto cos_2p = std::cos(2 * phi); //(1-2*cos(p)^2)
			const auto cot_t = one_div_sin_t*cos_t;
			DriftPreCalc(0) = 0.5*( (d_2+c_2*cos_2t)*cot_t + (d_2 - c_2)*cos_t*sin_t - 2 *c_d*one_div_sin_t*cos_p*sin_p);

			DriftPreCalc(1) = 0.5*cot_t*(c_d*one_div_sin_t*(1.0-3.0* cos_2p) - c_2*sin_2p);
		};

		BASIC_ALWAYS_INLINE void afterStepCheck(DependentVectorType& yi) const
		{
			yi(0) = periodicBoundaryTheta(yi(0));
			yi(1) = periodicBoundaryPhi(yi(1));
		};

		inline decltype(auto) getStart() noexcept
		{
			DependentVectorType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<precision> nd{ 0,1 };

			if (_Init.getUseRandomInitialParticleOrientation())
			{
				IndependentVectorType Orientation;
				for (std::size_t i = 0; i < 3; ++i)
					Orientation(i) = nd(rd);
				easyaxis = Orientation;
			}
			else
			{
				IndependentVectorType EulerAngles = _Init.getInitialParticleOrientation();
				IndependentVectorType Orientation;
				Orientation << 1, 0, 0;
				Matrix_3x3 tmp;
				const auto &a = EulerAngles[0]; //!< Alpha
				const auto &b = EulerAngles[1];	//!< Beta
				const auto &g = EulerAngles[2]; //!< Gamma
				tmp << cos(a)*cos(g) - sin(a)*cos(b)*sin(g), sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
					-cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
					sin(a)*sin(b), -cos(a)*sin(b), cos(b);
				easyaxis = tmp*Orientation;
			}

			if (_Init.getUseRandomInitialMagnetisationDir())
			{
				DependentVectorType MagDir;
				for (unsigned int i = 0; i < 2; ++i)
					MagDir(i) = nd(rd);
				Result = MagDir;
			}
			else
			{
				const IndependentVectorType tmp{ _Init.getInitialMagnetisationDirection() };
				IndependentVectorType z_axis;
				IndependentVectorType y_axis;
				IndependentVectorType x_axis;
				x_axis << 1.0, 0.0, 0.0;
				y_axis << 0.0, 1.0, 0.0;
				z_axis << 0.0, 0.0, 1.0;
				Result(0) = std::acos(tmp.dot(z_axis)); //Theta
				Result(1) = std::atan2(tmp.dot(y_axis), tmp.dot(x_axis)); //Phi
			}

			afterStepCheck(Result); //normalize if necessary
			return Result;
		};

		inline auto getWeighting() const noexcept
		{
			DependentVectorType scale{ DependentVectorType::Ones() };
			return (scale * _ParParams.getMagneticProperties().getSaturationMoment()).eval();
		};
	};
}

#include "Definitions/NeelRelaxationSpherical_Definitions.h"

#endif	// INC_NeelRelaxationSpherical_H
// end of Problems\NeelRelaxationSpherical.h
///---------------------------------------------------------------------------------------------------
