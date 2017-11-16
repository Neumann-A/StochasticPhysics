/* BrownAndNeelRelaxation Class
* Describes the combined neel and brown rotation
* Author: Alexander Neumann
* Date:   23.08.2015
*/

#pragma once

#ifndef _BROWNANDNEELRELAXATION_H_
#define _BROWNANDNEELRELAXATION_H_

#include "SDEFramework/GeneralSDEProblem.h"
#include "Results/SingleSimulationResult.h"

#include "Helpers/ParameterCalculatorBrownAndNeel.h"


namespace Problems
{
	namespace detail
	{
		template <bool SimpleModel = false>
		struct BrownStochasticMatrixSelector
		{
			template<typename Problem, typename DependentType>
			BASIC_ALWAYS_INLINE static auto SelectImpl(Problem&& prob, DependentType&& yi) noexcept
			{
				return prob.getStochasticMatrixFull(std::forward<DependentType>(yi));
			}
		};

		template <>
		struct BrownStochasticMatrixSelector<true>
		{
			template<typename Problem, typename DependentType>
			BASIC_ALWAYS_INLINE static auto SelectImpl(Problem&& prob, DependentType&& yi) noexcept
			{
				return prob.getStochasticMatrixSimplified(std::forward<DependentType>(yi));
			}
		};

		template <bool SimpleModel = false>
		struct BrownDriftSelector
		{
			template<typename Problem, typename DependentType>
			BASIC_ALWAYS_INLINE static auto SelectImpl(Problem&& prob, DependentType&& yi) noexcept
			{
				return prob.getStratonovichtoItoFull(std::forward<DependentType>(yi));
			}
		};

		template <>
		struct BrownDriftSelector<true>
		{
			template<typename Problem, typename DependentType>
			BASIC_ALWAYS_INLINE static auto SelectImpl(Problem&& prob, DependentType&& yi) noexcept
			{
				return prob.getStratonovichtoItoSimplified(std::forward<DependentType>(yi));
			}
		};

	}
	/*For constexpr to be useable in class definition they have to be defined before the class
	* can not put them into the class because the initialisation of the variables is done after
	* the class is completed so it will always give an error that the variables are not defined
	* if they are within the class
	*/
	constexpr static struct BrownAndNeelDimension : public GeneralSDEDimension<6, 3, 6> //thats pretty handy
	{ } BrownAndNeelDimensionVar; //too get the memory space

	template<typename precision, typename aniso, bool SimpleModel = false>
	class BrownAndNeelRelaxation : public GeneralSDEProblem<BrownAndNeelRelaxation<precision, aniso>>
	{
		template<bool test>
		friend struct detail::BrownStochasticMatrixSelector;
		template<bool test>
		friend struct detail::BrownDriftSelector;

	public: //Public Typedefs
		typedef BrownAndNeelRelaxation<precision, aniso>														ThisClass;
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



	private:
		typedef typename Traits::SubProblemMatrix																Matrix3x3;
		typedef typename Traits::SubVector																		Vec3D;
		typedef typename Traits::OutputType																		OutputType;

		template<typename T>
		using BaseMatrixType = typename Traits::template BaseMatrixType<T>;

	private:
		//Function pointers to include different cases
		StochasticMatrixType(BrownAndNeelRelaxation<precision, aniso>::* const toStochasticMatrix)(const DependentType& yi) const noexcept = nullptr;
		DeterministicType(BrownAndNeelRelaxation<precision, aniso>::* const toDrift)(const DependentType& yi) const noexcept = nullptr;

		const Helpers::BrownAndNeelMixedParams<Precision> _ParamHelper;

		//TODO: Move those to private section; Refactor code a bit;
	public:
		const UsedProperties		_ParParams;
		const InitSettings          _Init;
		const ProblemSettings		mProblemSettings;
		const Anisotropy			mAnisotropy;

	private:

		//The different cases
		inline auto getStratonovichtoItoFull(const DependentType& yi) const noexcept->DeterministicType;
		inline auto getStratonovichtoItoSimplified(const DependentType& yi) const noexcept->DeterministicType;

		inline auto getStochasticMatrixFull(const DependentType& yi) const noexcept->StochasticMatrixType;
		inline auto getStochasticMatrixSimplified(const DependentType& yi) const noexcept->StochasticMatrixType;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		//explicit BrownAndNeelRelaxation(const ProblemSettings& ProbSettings, SimulationParameters& Properties);
		explicit BrownAndNeelRelaxation(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init);

		BASIC_ALWAYS_INLINE auto getStochasticMatrix(const DependentType& yi) const noexcept-> StochasticMatrixType;
		BASIC_ALWAYS_INLINE auto getDrift(const DependentType& yi) const noexcept-> DeterministicType;
		BASIC_ALWAYS_INLINE auto getDeterministicVector(const DependentType& yi, const IndependentType& xi) const noexcept->DeterministicType;

		BASIC_ALWAYS_INLINE void finishCalculations(DependentType& yi) const noexcept;

		BASIC_ALWAYS_INLINE void prepareCalculations(DependentType& yi) const noexcept {};

		//inline decltype(auto) getStart() const noexcept;

		inline decltype(auto) getStart(const InitSettings& Init) const noexcept;

		static auto getWeighting(const UsedProperties &Properties) noexcept
		{
			DependentType scale{ DependentType::Ones() };

			scale.template tail<3>() *= Properties.getMagneticProperties().getSaturationMoment(); // Neel Direction Vector  

			return scale;
		};

		template<typename Derived>
		BASIC_ALWAYS_INLINE OutputType calculateOutputResult(const BaseMatrixType<Derived>& yi) const noexcept
		{
			return static_cast<const Derived&>(yi);
		}

		template<typename Derived>
		static BASIC_ALWAYS_INLINE void normalize(BaseMatrixType<Derived>& yi) noexcept
		{
			yi.template head<3>().normalize();
			yi.template tail<3>().normalize();
		};

	};
}
#include "BrownAndNeelRelaxation.inl"
#include "Definitions/BrownAndNeel_Definitions.h"


#endif //_BROWNANDNEELRELAXATION_H_