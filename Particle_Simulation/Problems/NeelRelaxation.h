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
		typedef typename Traits::DeterministicType														DeterministicType;
		typedef typename Traits::DependentType															DependentType;
		typedef typename Traits::IndependentType															IndependentType;
		typedef typename Traits::NoiseType																NoiseType;


		using OutputType = DependentType;

		template<typename T>
		using BaseMatrixType = typename Traits::template BaseMatrixType<T>;

		using JacobiMatrixType = typename Traits::JacobiMatrixType;
	private: // Important: Have often used Parameters at the top of the class defintion!
		
		//Particle Parameters
		Helpers::NeelParams<Precision>	mParams;
		//Helper Matrix
		DependentType mEasyAxis;
		
		const Anisotropy				mAnisotropy;
		const ProblemSettings			mProblemSettings;

		DependentType initEasyAxis(const InitSettings& Init)
		{
			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<precision> nd{ 0,1 };

			DependentType ea;
			//Init Particle Orientation (Easy Axis Init)
			if (Init.getUseRandomInitialParticleOrientation())
			{
				DependentType Orientation;
				for (unsigned int i = 0; i < 3; ++i)
					Orientation(i) = nd(rd);
				ea = Orientation;
				ea.normalize();
			}
			else
			{
				DependentType EulerAngles = Init.getInitialParticleOrientation();
				DependentType Orientation;
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

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		explicit NeelRelaxation(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
			GeneralSDEProblem<NeelRelaxation<precision, aniso>>(NeelDimensionVar),
			mParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
			mEasyAxis(initEasyAxis(Init)),
			mAnisotropy(Properties.getMagneticProperties()),
			mProblemSettings(ProbSettings), _ParParams(Properties), _Init(Init)
			 {
			assert(mEasyAxis.norm() <= 1.0 + 10.0 * std::numeric_limits<Precision>::epsilon() && mEasyAxis.norm() >= 1.0 - 10.0 * std::numeric_limits<Precision>::epsilon());
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const BaseMatrixType<Derived>& yi) const
		{
			StochasticMatrixType StochasticMatrix{ mParams.NeelNoise_H_Pre2*StochasticMatrixType::Identity() - (mParams.NeelNoise_H_Pre2*yi)*yi.transpose() };
			//StochasticMatrixType StochasticMatrix{ mParams.Damping*StochasticMatrixType::Identity() - (mParams.Damping*yi)*yi.transpose() };
			const auto yi2{ mParams.NeelNoise_H_Pre1*yi };
			//const auto yi2{ yi };
			//Crossproduct matrix
			StochasticMatrix(0,1) -= yi2(2);
			StochasticMatrix(0,2) += yi2(1);
			StochasticMatrix(1,0) += yi2(2);
			StochasticMatrix(1,2) -= yi2(0);
			StochasticMatrix(2,0) -= yi2(1);
			StochasticMatrix(2,1) += yi2(0);
			
			return StochasticMatrix;
			//return (mParams.NeelNoise_H_Pre1*StochasticMatrix);
			//return mParams.NeelFactor1*mParams.DriftPrefactor*StochasticMatrix;
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE DeterministicType getDrift(const BaseMatrixType<Derived>& yi) const
		{
			return (mParams.DriftPrefactor*yi).eval();
		};

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi) const
		{
			const auto Heff{ ((mAnisotropy.getAnisotropyField(yi,DeterministicType::Zero(),DeterministicType::Zero(),mEasyAxis) + xi)).eval() };
			
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

			//return (mParams.NeelFactor1*y_plus - mParams.NeelFactor2* (yi*yi.transpose() - JacobiMatrixType::Identity()))*Heff;

			
			return (mParams.NeelFactor1*yi.cross(Heff) - mParams.NeelFactor2*yi.cross(yi.cross(Heff))).eval();
			//return (yi.cross(Heff) - mParams.Damping*yi.cross(yi.cross(Heff))).eval();
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareJacobiCalculations(BaseMatrixType<Derived>& yi)
		{};

		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiDeterministic(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi,const Precision& dt) const
		{
			//Deterministc Jacobi Matrix
			const auto HeffJacobi{ mAnisotropy.getJacobiAnisotropyField(yi, mEasyAxis) };

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

			const auto Heff{ ((mAnisotropy.getAnisotropyField(yi,IndependentType::Zero(), IndependentType::Zero(),mEasyAxis) + xi)).eval() };

			//JacobiMatrixType JacobiDet{ mParams.NeelFactor1*m_plus*HeffJacobi - static_cast<Precision>(2.0)*mParams.Damping/dt*m_plus };
			JacobiMatrixType JacobiDet{ m_plus*HeffJacobi };
			{
				JacobiDet(0, 1) += Heff(2);
				JacobiDet(0, 2) -= Heff(1);
				JacobiDet(1, 0) -= Heff(2);
				JacobiDet(1, 2) += Heff(0);
				JacobiDet(2, 0) += Heff(1);
				JacobiDet(2, 1) -= Heff(0);
			}
			JacobiDet = mParams.NeelFactor1*JacobiDet - static_cast<Precision>(2.0)*mParams.Damping / dt*m_plus;

			return JacobiDet;
		}

		BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiStochastic(const NoiseType& dW) const
		{
			JacobiMatrixType JacobiSto{ JacobiMatrixType::Zero() };
			//Crossproduct matrix (c * dW) (minus due to minus sign in NeelNoise_H_Pre1)
			{
				const auto dw2{ -mParams.NeelNoise_H_Pre1*dW };

				JacobiSto(0, 1) -= dw2(2);
				JacobiSto(0, 2) += dw2(1);
				JacobiSto(1, 0) += dw2(2);
				JacobiSto(1, 2) -= dw2(0);
				JacobiSto(2, 0) -= dw2(1);
				JacobiSto(2, 1) += dw2(0);
				//std::cout << "Param\n" << mParams.NeelNoise_H_Pre1 << "\ndW\n" << dW << "\ndW2\n" << dw2 << "\nJacobiSto\n" << JacobiSto << std::endl;
			}
			return JacobiSto;
		}

		template<typename Derived>
		BASIC_ALWAYS_INLINE void finishJacobiCalculations(BaseMatrixType<Derived>& jacobi) const
		{

		}
		template<typename Derived, typename Derived2>
		BASIC_ALWAYS_INLINE auto getAllProblemParts(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi,
			const Precision& dt, const NoiseType& dW) const
		{
			//const auto nidotei = yi.dot(easyaxis).eval();
			
			//Deterministic Vector
			const auto Heff{ (mAnisotropy.getAnisotropyField(yi,mEasyAxis) + xi) }; // H_0 + H_K
			//const auto Pre_Heff{ mParams.NeelFactor1*Heff }; //will also be used later
			
			auto DetVec{ getDeterministicVector(yi,xi) };
			//const auto DetVec{ (yi.cross(Pre_Heff) - mParams.NeelFactor2*yi.cross(yi.cross(Heff))).eval() };
			//const auto DetVec{ (mParams.NeelFactor1*(yi.cross(Heff) - mParams.Damping*yi.cross(yi.cross(Heff)))).eval() };

			//Deterministc Jacobi Matrix
			const auto HeffJacobi{ mAnisotropy.getJacobiAnisotropyField(yi, mEasyAxis) };
			
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
			//JacobiMatrixType JacobiDet{ mParams.NeelFactor1*m_plus*HeffJacobi - static_cast<Precision>(2.0)*mParams.Damping/dt*m_plus };
			JacobiMatrixType JacobiDet{ m_plus*HeffJacobi };
			//std::cout << "yi:\n" << yi << "\n";
			//std::cout << "HeffJacobi:\n" << HeffJacobi << "\n";
			//std::cout << "m_plus*HeffJacobi:\n" << mParams.NeelFactor1*m_plus*HeffJacobi << "\n";
			//std::cout << "2 alpha m_plus / dt:\n" << 2.0*mParams.Damping*m_plus / dt << "\n";
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
			JacobiDet = mParams.NeelFactor1*JacobiDet - static_cast<Precision>(2.0)*mParams.Damping / dt*m_plus;

			//Stochastic Matrix ( m x (m x H_Noise))
			StochasticMatrixType StochasticMatrix{ getStochasticMatrix(yi) };
			
			//Stochastic Jacobi Matrix
			
			JacobiMatrixType JacobiSto{ JacobiMatrixType::Zero() };
			//Crossproduct matrix (c * dW) (minus due to minus sign in NeelNoise_H_Pre1)
			{
				const auto dw2{ -mParams.NeelNoise_H_Pre1*dW };
				
				JacobiSto(0, 1) -= dw2(2);
				JacobiSto(0, 2) += dw2(1);
				JacobiSto(1, 0) += dw2(2);
				JacobiSto(1, 2) -= dw2(0);
				JacobiSto(2, 0) -= dw2(1);
				JacobiSto(2, 1) += dw2(0);
				//std::cout << "Param\n" << mParams.NeelNoise_H_Pre1 << "\ndW\n" << dW << "\ndW2\n" << dw2 << "\nJacobiSto\n" << JacobiSto << std::endl;
			}
			
			return std::make_tuple(std::move(DetVec.eval()), std::move(JacobiDet.eval()), std::move(StochasticMatrix), std::move(JacobiSto.eval()));
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareCalculations(const BaseMatrixType<Derived>& yi) const noexcept{};

		template<typename Derived>
		BASIC_ALWAYS_INLINE void prepareJacobiCalculations(const BaseMatrixType<Derived>& yi) const noexcept {};

		template<typename Derived>
		BASIC_ALWAYS_INLINE void finishCalculations(BaseMatrixType<Derived>& yi) const
		{
			//yi.normalize();
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE OutputType calculateOutputResult(const BaseMatrixType<Derived>& yi) const noexcept
		{
			return static_cast<const Derived&>(yi);
		}

		inline decltype(auto) getStart(const InitSettings& Init) noexcept
		{
			DependentType Result;

			std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
			std::normal_distribution<precision> nd{ 0,1 };

			//Init Magnetisation Direction
			if (Init.getUseRandomInitialMagnetisationDir())
			{
				DependentType MagDir;
				for (unsigned int i = 0; i < 3; ++i)
					MagDir(i) = nd(rd);
				Result = MagDir;
			}
			else
			{
				Result = Init.getInitialMagnetisationDirection();
			}

			finishCalculations(Result); //normalize if necessary
			return Result;
		};

		static auto getWeighting(const UsedProperties &Properties) noexcept
		{
			OutputType scale{ OutputType::Ones() };
			return (scale * Properties.getMagneticProperties().getSaturationMoment()).eval();
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

#include "Definitions/NeelRelaxation_Definitions.h"

#endif	// INC_NeelRelaxation_H
// end of NeelRelaxation.h
///---------------------------------------------------------------------------------------------------
