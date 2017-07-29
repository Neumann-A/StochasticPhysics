///---------------------------------------------------------------------------------------------------
// file:		NeelRelaxation.h
//
// summary: 	Declares the neel relaxation class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 07.02.2017

#ifndef INC_NeelRelaxation_H
#define INC_NeelRelaxation_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <tuple>
#include <random>

#include "../SDEFramework/GeneralSDEProblem.h"
#include "Helpers/ParameterCalculatorNeel.h"

namespace Problems
{
	constexpr static struct NeelDimension : GeneralSDEDimension<3, 3, 3> //thats pretty handy
	{ } NeelDimensionVar; //too get the memory space (else the compiler will optimize it away)

	template<typename precision, typename aniso>
	class NeelRelaxation :
		public GeneralSDEProblem <NeelRelaxation<precision, aniso>>
	{
	public:
		typedef NeelRelaxation<precision, aniso>																ThisClass;
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

		using JacobiMatrixType = typename Traits::JacobiMatrixType;
	private: // Important: Have often used Parameters at the top of the class defintion!
		
		//Particle Parameters
		Helpers::NeelParams<Precision>	_Params;
		//Helper Matrix
		DependentVectorType _easyaxis;
		
		const Anisotropy				_Anisotropy;
		const ProblemSettings			_ProbSet;

		DependentVectorType initEasyAxis(const InitSettings& Init)
		{
			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<precision> nd{ 0,1 };

			DependentVectorType ea;
			//Init Particle Orientation (Easy Axis Init)
			if (Init.getUseRandomInitialParticleOrientation())
			{
				DependentVectorType Orientation;
				for (unsigned int i = 0; i < 3; ++i)
					Orientation(i) = nd(rd);
				ea = Orientation;
				ea.normalize();
			}
			else
			{
				DependentVectorType EulerAngles = Init.getInitialParticleOrientation();
				DependentVectorType Orientation;
				Orientation << 1, 0, 0;
				StochasticMatrixType tmp;
				const auto &a = EulerAngles[0]; //!< Alpha
				const auto &b = EulerAngles[1];	//!< Beta
				const auto &g = EulerAngles[2]; //!< Gamma
				tmp << cos(a)*cos(g) - sin(a)*cos(b)*sin(g), sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
					-cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
					sin(a)*sin(b), -cos(a)*sin(b), cos(b);
				ea = tmp*Orientation;
			}
			return ea;
		}

	public:
		const UsedProperties		_ParParams;
		const InitSettings          _Init;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		explicit NeelRelaxation(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
			GeneralSDEProblem<NeelRelaxation<precision, aniso>>(NeelDimensionVar),
			_Params(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
			_easyaxis(initEasyAxis(Init)),
			_Anisotropy(Properties.getMagneticProperties()),
			_ProbSet(ProbSettings), _ParParams(Properties), _Init(Init)
			 {
			assert(_easyaxis.norm() <= 1.0 + 10.0 * std::numeric_limits<Precision>::epsilon() && _easyaxis.norm() >= 1.0 - 10.0 * std::numeric_limits<Precision>::epsilon());
		};

		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const DependentVectorType& yi) const
		{
			StochasticMatrixType StochasticMatrix{ _Params.NeelNoise_H_Pre2*StochasticMatrixType::Identity() - (_Params.NeelNoise_H_Pre2*yi)*yi.transpose() };
			//StochasticMatrixType StochasticMatrix{ _Params.Damping*StochasticMatrixType::Identity() - (_Params.Damping*yi)*yi.transpose() };
			const auto yi2{ _Params.NeelNoise_H_Pre1*yi };
			//const auto yi2{ yi };
			//Crossproduct matrix
			StochasticMatrix(0,1) -= yi2(2);
			StochasticMatrix(0,2) += yi2(1);
			StochasticMatrix(1,0) += yi2(2);
			StochasticMatrix(1,2) -= yi2(0);
			StochasticMatrix(2,0) -= yi2(1);
			StochasticMatrix(2,1) += yi2(0);
			
			return StochasticMatrix;
			//return (_Params.NeelNoise_H_Pre1*StochasticMatrix);
			//return _Params.NeelFactor1*_Params.DriftPrefactor*StochasticMatrix;
		};

		BASIC_ALWAYS_INLINE DeterministicVectorType getDrift(const DependentVectorType& yi) const
		{
			return (_Params.DriftPrefactor*yi).eval();
		};

		BASIC_ALWAYS_INLINE DeterministicVectorType getDeterministicVector(const DependentVectorType& yi, const IndependentVectorType& xi) const
		{
			const auto Heff{ ((_Anisotropy.getAnisotropyField(yi,_easyaxis) + xi)).eval() };
			
			//JacobiMatrixType y_plus{ JacobiMatrixType::Zero() };
			//{
			//	const auto& y{ yi };
			//	y_plus(0, 1) = -y(2);
			//	y_plus(0, 2) = +y(1);
			//	y_plus(1, 0) = +y(2);
			//	y_plus(1, 2) = -y(0);
			//	y_plus(2, 0) = -y(1);
			//	y_plus(2, 1) = +y(0);
			//}

			//return (_Params.NeelFactor1*y_plus - _Params.NeelFactor2* (yi*yi.transpose() - JacobiMatrixType::Identity()))*Heff;

			
			return (_Params.NeelFactor1*yi.cross(Heff) - _Params.NeelFactor2*yi.cross(yi.cross(Heff))).eval();
			//return (yi.cross(Heff) - _Params.Damping*yi.cross(yi.cross(Heff))).eval();
		};

		BASIC_ALWAYS_INLINE auto getAllProblemParts(const DependentVectorType& yi, const IndependentVectorType& xi,
			const Precision& dt, const NoiseVectorType& dW) const
		{
			//const auto nidotei = yi.dot(easyaxis).eval();
			
			//Deterministic Vector
			const auto Heff{ (_Anisotropy.getAnisotropyField(yi,_easyaxis) + xi) }; // H_0 + H_K
			//const auto Pre_Heff{ _Params.NeelFactor1*Heff }; //will also be used later
			
			auto DetVec{ getDeterministicVector(yi,xi) };
			//const auto DetVec{ (yi.cross(Pre_Heff) - _Params.NeelFactor2*yi.cross(yi.cross(Heff))).eval() };
			//const auto DetVec{ (_Params.NeelFactor1*(yi.cross(Heff) - _Params.Damping*yi.cross(yi.cross(Heff)))).eval() };

			//Deterministc Jacobi Matrix
			const auto HeffJacobi{ _Anisotropy.getJacobiAnisotropyField(yi, _easyaxis) };
			
			JacobiMatrixType m_plus{ JacobiMatrixType::Zero() };
			{
				const auto& m{ yi };
				m_plus(0, 1) = -m(2);
				m_plus(0, 2) = +m(1);
				m_plus(1, 0) = +m(2);
				m_plus(1, 2) = -m(0);
				m_plus(2, 0) = -m(1);
				m_plus(2, 1) = +m(0);
			}			
			//JacobiMatrixType JacobiDet{ _Params.NeelFactor1*m_plus*HeffJacobi - static_cast<Precision>(2.0)*_Params.Damping/dt*m_plus };
			JacobiMatrixType JacobiDet{ m_plus*HeffJacobi };
			//std::cout << "yi:\n" << yi << "\n";
			//std::cout << "HeffJacobi:\n" << HeffJacobi << "\n";
			//std::cout << "m_plus*HeffJacobi:\n" << _Params.NeelFactor1*m_plus*HeffJacobi << "\n";
			//std::cout << "2 alpha m_plus / dt:\n" << 2.0*_Params.Damping*m_plus / dt << "\n";
			//std::cout << "Jac Det before Pre_Heff:\n" << JacobiDet << "\n";
			//std::cout << "Pre_Heff:\n" << Pre_Heff << "\n";
			{
				JacobiDet(0, 1) += Heff(2);
				JacobiDet(0, 2) -= Heff(1);
				JacobiDet(1, 0) -= Heff(2);
				JacobiDet(1, 2) += Heff(0);
				JacobiDet(2, 0) += Heff(1);
				JacobiDet(2, 1) -= Heff(0);
			}
			JacobiDet = _Params.NeelFactor1*JacobiDet - static_cast<Precision>(2.0)*_Params.Damping / dt*m_plus;

			//Stochastic Matrix ( m x (m x H_Noise))
			StochasticMatrixType StochasticMatrix{ getStochasticMatrix(yi) };
			
			//Stochastic Jacobi Matrix
			
			JacobiMatrixType JacobiSto{ JacobiMatrixType::Zero() };
			//Crossproduct matrix (c * dW) (minus due to minus sign in NeelNoise_H_Pre1)
			{
				const auto dw2{ -_Params.NeelNoise_H_Pre1*dW };
				
				JacobiSto(0, 1) -= dw2(2);
				JacobiSto(0, 2) += dw2(1);
				JacobiSto(1, 0) += dw2(2);
				JacobiSto(1, 2) -= dw2(0);
				JacobiSto(2, 0) -= dw2(1);
				JacobiSto(2, 1) += dw2(0);
				//std::cout << "Param\n" << _Params.NeelNoise_H_Pre1 << "\ndW\n" << dW << "\ndW2\n" << dw2 << "\nJacobiSto\n" << JacobiSto << std::endl;
			}
			
			return std::make_tuple(std::move(DetVec.eval()), std::move(JacobiDet.eval()), std::move(StochasticMatrix), std::move(JacobiSto.eval()));
		};

		BASIC_ALWAYS_INLINE void prepareNextStep(DependentVectorType& yi) const noexcept{};

		BASIC_ALWAYS_INLINE void afterStepCheck(DependentVectorType& yi) const
		{
			yi.normalize();
		};

		inline decltype(auto) getStart() noexcept
		{
			DependentVectorType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<precision> nd{ 0,1 };

			//Init Magnetisation Direction
			if (_Init.getUseRandomInitialMagnetisationDir())
			{
				DependentVectorType MagDir;
				for (unsigned int i = 0; i < 3; ++i)
					MagDir(i) = nd(rd);
				Result = MagDir;
			}
			else
			{
				Result = _Init.getInitialMagnetisationDirection();
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

#include "Definitions/NeelRelaxation_Definitions.h"

#endif	// INC_NeelRelaxation_H
// end of NeelRelaxation.h
///---------------------------------------------------------------------------------------------------
