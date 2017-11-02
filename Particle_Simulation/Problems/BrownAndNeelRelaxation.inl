/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

//#include "BrownAndNeelRelaxation.h" //File is included by Header!

//IMPORTANT: NEED TO RECHECK ALL SIGNS -> CREATE TEST CASES!

#ifdef _BROWNANDNEELRELAXATION_H_

namespace Problems
{
	template<typename precision, typename aniso, bool SimpleModel>
	BrownAndNeelRelaxation<precision, aniso, SimpleModel>::BrownAndNeelRelaxation(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
		GeneralSDEProblem<BrownAndNeelRelaxation<precision, aniso>>(BrownAndNeelDimensionVar),
		//toStochasticMatrix(ProbSettings.getUseSimpleModel() ? &BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixSimplified : &BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixFull),
		//toDrift(ProbSettings.getUseSimpleModel() ? &BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoSimplified : &BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoFull),
		_ParamHelper(Properties),
		_ParParams(Properties), _Init(Init), mProblemSettings(ProbSettings),
		mAnisotropy(Properties.getMagneticProperties())
	{};


	//Get the stoachstig matrix
	template<typename precision, typename aniso, bool SimpleModel>
	BASIC_ALWAYS_INLINE auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStochasticMatrix(const DependentType& yi) const noexcept-> StochasticMatrixType
	{
		return detail::BrownStochasticMatrixSelector<SimpleModel>::SelectImpl(*this, yi);
		//return (this->*toStochasticMatrix)(yi);
	};

	//Actual Calculation of the Stochastic Matrix; Full model with noise coupling
	template<typename precision, typename aniso, bool SimpleModel>
	inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStochasticMatrixFull(const DependentType& yi) const noexcept -> StochasticMatrixType
	{
		const auto& ni{ (yi.template head<3>()) };// Brown Direction Vector 
		const auto& ei{ (yi.template tail<3>()) };// Neel Direction Vector  

		StochasticMatrixType StochasticMatrix; // Return Matrix

		//Block Expresions
		auto Brown_F{ StochasticMatrix.template topLeftCorner<3, 3>() };
		auto Brown_H{ StochasticMatrix.template topRightCorner<3, 3>() };
		auto Neel_F{ StochasticMatrix.template bottomLeftCorner<3, 3>() };
		auto Neel_H{ StochasticMatrix.template bottomRightCorner<3, 3>() };

		/* BEGIN Mixed Terms describing the coupling */
		Brown_H = _ParamHelper.Brown_H_Noise()*(ei*ni.transpose() - Matrix3x3::Constant(ni.dot(ei)));
		Neel_F = _ParamHelper.Neel_F_Noise()*Brown_H.transpose();
		/* End Mixed Terms */

		/* BEGIN Brown Rotation */
		Brown_F = _ParamHelper.Brown_F_Noise()*(ni*ni.transpose() - Matrix3x3::Identity());
		/* END Brown Rotation */

		/* BEGIN Neel Rotation*/
		{
			auto outerei = (ei*ei.transpose() - Matrix3x3::Identity()).eval();
			Neel_H = (_ParamHelper.Brown_H_Noise() + _ParamHelper.NeelPre2_H_Noise())*outerei;
			auto eitmp{ (_ParamHelper.NeelPre1_H_Noise()*ei).eval() };
			Neel_H(0, 1) += eitmp(2);
			Neel_H(0, 2) -= eitmp(1);
			Neel_H(1, 0) -= eitmp(2);
			Neel_H(1, 2) += eitmp(0);
			Neel_H(2, 0) += eitmp(1);
			Neel_H(2, 1) -= eitmp(0);
		}
		/* END Neel Rotation*/

		return StochasticMatrix;
	}

	//Simplified version; ignoring noise coupling between brown and neel
	template<typename precision, typename aniso, bool SimpleModel>
	inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStochasticMatrixSimplified(const DependentType& yi) const noexcept-> StochasticMatrixType
	{
		const auto& ni{ yi.template head<3>() };// Brown Direction Vector 
		const auto& ei{ yi.template tail<3>() };// Neel Direction Vector  
		
		StochasticMatrixType StochasticMatrix;
		auto Brown_F = StochasticMatrix.template topLeftCorner<3, 3>();
		auto Brown_H = StochasticMatrix.template topRightCorner<3, 3>();
		auto Neel_F = StochasticMatrix.template bottomLeftCorner<3, 3>();
		auto Neel_H = StochasticMatrix.template bottomRightCorner<3, 3>();

		/* BEGIN Brown Rotation */
		Brown_F = _ParamHelper.Brown_F_Noise()*(ni*ni.transpose() - Matrix3x3::Identity());
		Brown_H = Matrix3x3::Zero();
		/* END Brown Rotation */

		/* BEGIN Neel Rotation*/
		{
			auto outerei = (ei*ei.transpose() - Matrix3x3::Identity()).eval();
			Neel_F = Matrix3x3::Zero();
			Neel_H = (_ParamHelper.Brown_H_Noise() + _ParamHelper.NeelPre2_H_Noise())*outerei;

			auto eitmp{ _ParamHelper.NeelPre1_H_Noise()*ei };
			Neel_H(0, 1) += eitmp(2);
			Neel_H(0, 2) -= eitmp(1);
			Neel_H(1, 0) -= eitmp(2);
			Neel_H(1, 2) += eitmp(0);
			Neel_H(2, 0) += eitmp(1);
			Neel_H(2, 1) -= eitmp(0);
		}
		/* END Neel Rotation*/

		return StochasticMatrix;
	}

	//Gets the Drift term
	template<typename precision, typename aniso, bool SimpleModel>
	BASIC_ALWAYS_INLINE auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getDrift(const DependentType& yi) const noexcept-> DeterministicType
	{
		return detail::BrownDriftSelector<SimpleModel>::SelectImpl(*this, yi);
		//return 	((this->*toDrift)(yi));
	};

	template<typename precision, typename aniso, bool SimpleModel>
	inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStratonovichtoItoSimplified(const DependentType& yi) const noexcept-> DeterministicType
	{
		DeterministicType result;
		result.template head<3>() = _ParamHelper.min_a_2()*yi.template head<3>();
		result.template tail<3>() =	_ParamHelper.min__c_2_plus__b_plus_d__2()*yi.template tail<3>();
		return result;
	};

	template<typename precision, typename aniso, bool SimpleModel>
	inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStratonovichtoItoFull(const DependentType& yi) const noexcept -> DeterministicType
	{
		const auto& ni{ yi.template head<3>() };// Brown Direction Vector 
		const auto& ei{ yi.template tail<3>() };// Neel Direction Vector  

		//Correct Code
		DeterministicType tmp;

		const auto ni_ei_cross = ni.cross(ei).eval();
		tmp.template head<3>() = _ParamHelper.order1a()*ni + _ParamHelper.order2a()*ni_ei_cross + _ParamHelper.order3a()*ni.cross(ni_ei_cross);
		tmp.template tail<3>() = _ParamHelper.order1b()*ei + _ParamHelper.order3b()*ei.cross(ni_ei_cross);

		//Alternative just as fast, a bit more code
		//const auto ni_ei_cross = ni.cross(ei).eval();
		//const auto ni_ei_dot = (ni.dot(ei));
		//tmp.template head<3>() = _ParamHelper.order1a()*ni + _ParamHelper.order2a()*ni_ei_cross + _ParamHelper.order3a()*(ni-ei*ni_ei_dot);
		//tmp.template tail<3>() = _ParamHelper.order1b()*ei + _ParamHelper.order3b()*(ei-ni*ni_ei_dot);

		return tmp;
	};

	//Actual Calculation of the Deterministic Matrix (no approx needed) (no difference between simple and full model)
	template<typename precision, typename aniso, bool SimpleModel>
	BASIC_ALWAYS_INLINE auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getDeterministicVector(const DependentType& yi, const IndependentType& xi) const noexcept-> DeterministicType
	{
		//Faster than any 4D Version
		const auto& ni{ yi.template head<3>() }; // Brown Direction Vector 
		const auto& ei{ yi.template tail<3>() }; // Neel Direction Vector  
				
		DeterministicType result;
		auto Brown{ result.template head<3>() };
		auto Neel{ result.template tail<3>() };

		/* BEGIN Brown Rotation*/
		Brown = _ParamHelper.NeelBrownMixPre()*ni.cross(ei.cross(xi)) ;
		/* END Brown Rotation*/

		/* BEGIN Neel Rotation*/
		auto EffField{ (mAnisotropy.getAnisotropyField(ei, IndependentType::Zero(), IndependentType::Zero(), ni) + xi) };
		auto helper = EffField.cross(ei);
		Neel = _ParamHelper.NeelPrefactor1()*helper - (_ParamHelper.NeelPre2PlusMixPre())*ei.cross(helper);
		/* END Neel Rotation*/

		return result;
	}

	template<typename precision, typename aniso, bool SimpleModel>
	BASIC_ALWAYS_INLINE void BrownAndNeelRelaxation<precision, aniso, SimpleModel>::finishCalculations(DependentType& yi) const noexcept
	{
		yi.template head<3>().normalize();
		yi.template tail<3>().normalize();
	};

	template<typename precision, typename aniso, bool SimpleModel>
	inline decltype(auto) BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStart(const InitSettings& Init) const noexcept
	{
		DependentType Result;

		std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
		std::normal_distribution<precision> nd{ 0,1 };

		//for (unsigned int i = 0; i < dim::NumberOfDependentVariables;++i)
		//	tmpvec(i) = nd(rd);

		if (Init.getUseRandomInitialParticleOrientation())
		{
			Vec3D Orientation;
			for (unsigned int i = 0; i < 3; ++i)
				Orientation(i) = nd(rd);
			Result.template head<3>() = Orientation;
		}
		else
		{
			Vec3D EulerAngles = Init.getInitialParticleOrientation();
			Vec3D Orientation;
			Orientation << 1.0, 0.0, 0.0;
			Matrix3x3	tmp;
			const auto &a = EulerAngles[0]; //!< Alpha
			const auto &b = EulerAngles[1];	//!< Beta
			const auto &g = EulerAngles[2]; //!< Gamma
			tmp << cos(a)*cos(g) - sin(a)*cos(b)*sin(g), sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
				-cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
				sin(a)*sin(b), -cos(a)*sin(b), cos(b);
			Result.template head<3>() = tmp*Orientation;
		}

		if (Init.getUseRandomInitialMagnetisationDir())
		{
			Vec3D MagDir;
			for (unsigned int i = 0; i < 3; ++i)
				MagDir(i) = nd(rd);
			Result.template tail<3>() = MagDir;
		}
		else
		{
			Result.template tail<3>() = Init.getInitialMagnetisationDirection();
		}

		finishCalculations(Result); //normalize if necessary
		return Result;
	};
}
#endif //_BROWNANDNEELRELAXATION_H_

