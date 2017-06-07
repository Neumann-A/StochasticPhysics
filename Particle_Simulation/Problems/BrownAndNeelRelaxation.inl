/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

//#include "BrownAndNeelRelaxation.h" //File is included by Header!

#ifdef _BROWNANDNEELRELAXATION_H_

namespace Problems
{
	template<typename precision, typename aniso>
	BrownAndNeelRelaxation<precision, aniso>::BrownAndNeelRelaxation(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
		GeneralSDEProblem<BrownAndNeelRelaxation<precision, aniso>>(BrownAndNeelDimensionVar),
		toStochasticMatrix(ProbSettings.getUseSimpleModel() ? &BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixSimplified : &BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixFull),
		toDrift(ProbSettings.getUseSimpleModel() ? &BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoSimplified : &BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoFull),
		_ParamHelper(Properties),
		_ParParams(Properties), _Init(Init), _ProbSet(ProbSettings),
		_Anisotropy(Properties.getMagneticProperties().getSaturationMagnetisation(), Properties.getMagneticProperties().getAnisotropyConstants())
	{};


	//Get the stoachstig matrix
	template<typename precision, typename aniso>
	inline auto BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrix(const DependentVectorType& yi) const noexcept-> StochasticMatrixType
	{
		return (this->*toStochasticMatrix)(yi);
	};

	//Actual Calculation of the Stochastic Matrix; Full model with noise coupling
	template<typename precision, typename aniso>
	inline auto BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixFull(const DependentVectorType& yi) const noexcept -> StochasticMatrixType
	{
		auto& ni{ (yi.template head<3>()) };// Brown Direction Vector 
		auto& ei{ (yi.template tail<3>()) };// Neel Direction Vector  

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
			auto outerei = (ei*ei.transpose()).eval();
			Neel_H = _ParamHelper.Brown_H_Noise()*(outerei - Matrix3x3::Identity()) + _ParamHelper.NeelPre2_H_Noise()*outerei;
		}
		//See below to understand these strange assignment lines!
		auto eitmp{ (_ParamHelper.NeelPre1_H_Noise()*ei).eval() };
		StochasticMatrix(22) += eitmp(2);
		StochasticMatrix(23) -= eitmp(1);
		StochasticMatrix(27) -= eitmp(2);
		StochasticMatrix(29) += eitmp(0);
		StochasticMatrix(33) += eitmp(1);
		StochasticMatrix(34) -= eitmp(0);
		// Cross product matrix; Dont delete these lines; Better to understand whats going up above
		//Neel_H(1) -= eitmp2(2);
		//Neel_H(2) += eitmp2(1);
		//Neel_H(3) += eitmp2(2);
		//Neel_H(5) -= eitmp2(0);
		//Neel_H(6) -= eitmp2(1);
		//Neel_H(7) += eitmp2(0);
	
		/* END Neel Rotation*/

		return StochasticMatrix;
	}

	//Simplified version; ignoring noise coupling between brown and neel
	template<typename precision, typename aniso>
	inline auto BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixSimplified(const DependentVectorType& yi) const noexcept-> StochasticMatrixType
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
		auto outerei = (ei*ei.transpose()).eval();
		Neel_F = Matrix3x3::Zero();
		Neel_H = _ParamHelper.Brown_H_Noise()*(outerei - Matrix3x3::Identity()) + _ParamHelper.NeelPre2_H_Noise()*outerei;

		auto eitmp{ _ParamHelper.NeelPre1_H_Noise()*ei };
		StochasticMatrix(27) -= eitmp(2);
		StochasticMatrix(33) += eitmp(1);
		StochasticMatrix(22) += eitmp(2);
		StochasticMatrix(34) -= eitmp(0);
		StochasticMatrix(23) -= eitmp(1);
		StochasticMatrix(29) += eitmp(0);
		// Cross product matrix; Dont delete these lines; Better to understand whats going up above
		//Neel_H(1) -= eitmp2(2);
		//Neel_H(2) += eitmp2(1);
		//Neel_H(3) += eitmp2(2);
		//Neel_H(5) -= eitmp2(0);
		//Neel_H(6) -= eitmp2(1);
		//Neel_H(7) += eitmp2(0);
		/* END Neel Rotation*/

		return StochasticMatrix;
	}

	//Gets the Drift term
	template<typename precision, typename aniso>
	BASIC_ALWAYS_INLINE auto BrownAndNeelRelaxation<precision, aniso>::getDrift(const DependentVectorType& yi) const noexcept-> DeterministicVectorType
	{
		return 	((this->*toDrift)(yi));
	};

	template<typename precision, typename aniso>
	inline auto BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoSimplified(const DependentVectorType& yi) const noexcept-> DeterministicVectorType
	{
		DeterministicVectorType result;
		//result << _ParamHelper.half_min_a_2()*yi.template head<3>(), _ParamHelper.half_min_b_2_min_c_2_plus_d_2()*yi.template tail<3>();
		result.template head<3>() = _ParamHelper.half_min_a_2()*yi.template head<3>();
		result.template tail<3>() =	_ParamHelper.half_min_b_2_min_c_2_plus_d_2()*yi.template tail<3>();
		return result;
	};

	template<typename precision, typename aniso>
	inline auto BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoFull(const DependentVectorType& yi) const noexcept -> DeterministicVectorType
	{
		//TODO: nochmal überprüfen
		const auto& ni{ yi.template head<3>() };// Brown Direction Vector 
		const auto& ei{ yi.template tail<3>() };// Neel Direction Vector  

		// order1a, order1b; // Linear terms in yi
		// order2a; // quadratic terms in yi (only upper part)
		
		DeterministicVectorType tmp;
		tmp.template head<3>() = _ParamHelper.order1a()*ni + _ParamHelper.order2a()*ni.cross(ei);
		tmp.template tail<3>() = _ParamHelper.order1b()*ei;

		DeterministicVectorType order3; // cubic terms in yi (no matrix formulation found so far! (kind of annoying);  should be somehow optimized)
		order3(0) = _ParamHelper.b_2()*(yi(4)*(yi(3)*yi(4) + 0.5*yi(1)*yi(3) - 0.5*yi(0)*yi(4) + yi(1)*yi(4)) + yi(5)*(yi(0)*yi(3) + 0.5*yi(2)*yi(3) + yi(1)*yi(4) + yi(2)*yi(4) - 0.5*yi(0)*yi(5) + yi(2)*yi(5))) - _ParamHelper.bc_div_2()*(yi(0)*yi(3)*yi(4) + yi(1)*yi(4)*yi(4) + yi(0)*yi(3)*yi(5) + yi(1)*yi(4)*yi(5) + yi(2)*yi(4)*yi(5) + yi(2)*yi(5)*yi(5));
		order3(1) = _ParamHelper.b_2()*(yi(4)*(0.5*yi(0)*yi(3) + yi(1)*yi(3) - yi(0)*yi(4) + 0.5*yi(1)*yi(4)) + yi(5)*(yi(0)*yi(3) + yi(2)*yi(3) + yi(1)*yi(4) + 0.5*yi(2)*yi(4) - yi(0)*yi(5) + yi(2)*yi(5))) - _ParamHelper.bc_div_2()*(yi(0)*yi(3)*yi(4) - yi(0)*yi(4)*yi(4) + yi(0)*yi(3)*yi(5) + yi(2)*yi(3)*yi(5) + yi(1)*yi(4)*yi(5) - yi(0)*yi(5)*yi(5) + yi(2)*yi(5)*yi(5));
		order3(2) = _ParamHelper.b_2()*(yi(4)*(yi(3)*yi(4) + yi(1)*yi(3) - yi(0)*yi(4) + yi(1)*yi(4)) + yi(5)*(0.5*yi(0)*yi(3) + yi(2)*yi(3) + 0.5*yi(1)*yi(4) + yi(2)*yi(4) - yi(0)*yi(5) + 0.5*yi(2)*yi(5))) - _ParamHelper.bc_div_2()*(yi(0)*yi(3)*yi(4) + yi(1)*yi(3)*yi(4) - yi(0)*yi(4)*yi(4) + yi(1)*yi(4)*yi(4) + yi(2)*yi(3)*yi(5) + yi(2)*yi(4)*yi(5) - yi(0)*yi(5)*yi(5));

		order3(3) = _ParamHelper.b_2() * (1.5*yi(0)*yi(1)*yi(3) - 0.5*yi(1)*yi(1)*yi(3) + 1.5*yi(0)*yi(2)*yi(3) - 0.5*yi(2)*yi(2)*yi(3) + 0.5*yi(0)*yi(1)*yi(4) + 1.5*yi(1)*yi(1)*yi(4) + 1.5*yi(1)*yi(2)*yi(4) + 0.5*yi(0)*yi(2)*yi(5) + 1.5*yi(1)*yi(2)*yi(5) + 1.5*yi(2)*yi(2)*yi(5)) - _ParamHelper.a_b_half()*(yi(0)*yi(1)*yi(3) + yi(0)*yi(2)*yi(3) + yi(1)*yi(1)*yi(4) + yi(1)*yi(2)*yi(4) + yi(1)*yi(2)*yi(5) + yi(2)*yi(2)*yi(5));
		order3(4) = _ParamHelper.b_2() * (0.5*yi(0)*yi(1)*yi(3) - 1.5*yi(1)*yi(1)*yi(3) + 1.5*yi(0)*yi(2)*yi(3) - 1.5*yi(2)*yi(2)*yi(3) + 1.5*yi(0)*yi(1)*yi(4) + 0.5*yi(1)*yi(1)*yi(4) + 1.5*yi(1)*yi(2)*yi(4) + 1.5*yi(0)*yi(2)*yi(5) + 0.5*yi(1)*yi(2)*yi(5) + 1.5*yi(2)*yi(2)*yi(5)) + _ParamHelper.a_b_half()*(yi(1)*yi(1)*yi(3) - yi(0)*yi(2)*yi(3) + yi(2)*yi(2)*yi(3) - yi(0)*yi(1)*yi(4) - yi(1)*yi(2)*yi(4) - yi(0)*yi(2)*yi(5) - yi(2)*yi(2)*yi(5));
		order3(5) = _ParamHelper.b_2() * (1.5*yi(0)*yi(1)*yi(3) - 1.5*yi(1)*yi(1)*yi(3) + 0.5*yi(0)*yi(2)*yi(3) - 1.5*yi(2)*yi(2)*yi(3) + 1.5*yi(0)*yi(1)*yi(4) + 1.5*yi(1)*yi(1)*yi(4) + 0.5*yi(1)*yi(2)*yi(4) + 1.5*yi(0)*yi(2)*yi(5) + 1.5*yi(1)*yi(2)*yi(5) + 0.5*yi(2)*yi(2)*yi(5)) - _ParamHelper.a_b_half()*(yi(0)*yi(1)*yi(3) - yi(1)*yi(1)*yi(3) - yi(2)*yi(2)*yi(3) + yi(0)*yi(1)*yi(4) + yi(1)*yi(1)*yi(4) + yi(0)*yi(2)*yi(5) + yi(1)*yi(2)*yi(5));;

		return (0.5*(tmp + order3));
	};

	//Actual Calculation of the Deterministic Matrix (no approx needed) (no difference between simple and full model)
	template<typename precision, typename aniso>
	inline auto BrownAndNeelRelaxation<precision, aniso>::getDeterministicVector(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept-> DeterministicVectorType
	{
		//Faster than any 4D Version
		const auto& ni{ yi.template head<3>() }; // Brown Direction Vector 
		const auto& ei{ yi.template tail<3>() }; // Neel Direction Vector  
				
		DeterministicVectorType result;
		auto Brown{ result.template head<3>() };
		auto Neel{ result.template tail<3>() };
		/* BEGIN Brown Rotation*/
		Brown = _ParamHelper.NeelBrownMixPre()*ni.cross(ei.cross(xi)) ;
		/* END Brown Rotation*/

		/* BEGIN Neel Rotation*/
		auto EffField{ (_Anisotropy.getEffectiveField(ei, ni) + xi) };
		auto helper = EffField.cross(ei);
		//Neel = (_ParamHelper.NeelPrefactor1()*EffField.cross(ei) + _ParamHelper.NeelPrefactor2()*ei.cross(ei.cross(EffField)) + _ParamHelper.NeelBrownMixPre() *ei.cross(ei.cross(EffField))) ;
		//Neel = _ParamHelper.NeelPrefactor1()*EffField.cross(ei) + (_ParamHelper.NeelPrefactor2() + _ParamHelper.NeelBrownMixPre())*ei.cross(ei.cross(EffField));
		Neel = _ParamHelper.NeelPrefactor1()*helper - (_ParamHelper.NeelPre2PlusMixPre())*ei.cross(helper);
		/* END Neel Rotation*/

		return result;
	}

	template<typename precision, typename aniso>
	inline void BrownAndNeelRelaxation<precision, aniso>::afterStepCheck(DependentVectorType& yi) const noexcept
	{
		yi.template head<3>().normalize();
		yi.template tail<3>().normalize();
	};

	template<typename precision, typename aniso>
	inline decltype(auto) BrownAndNeelRelaxation<precision, aniso>::getStart() const noexcept
	{
		DependentVectorType Result;

		std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
		std::normal_distribution<precision> nd{ 0,1 };

		//for (unsigned int i = 0; i < dim::NumberOfDependentVariables;++i)
		//	tmpvec(i) = nd(rd);

		if (_Init.getUseRandomInitialParticleOrientation())
		{
			Vec3D Orientation;
			for (unsigned int i = 0; i < 3; ++i)
				Orientation(i) = nd(rd);
			Result.template head<3>() = Orientation;
		}
		else
		{
			Vec3D EulerAngles = _Init.getInitialParticleOrientation();
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

		if (_Init.getUseRandomInitialMagnetisationDir())
		{
			Vec3D MagDir;
			for (unsigned int i = 0; i < 3; ++i)
				MagDir(i) = nd(rd);
			Result.template tail<3>() = MagDir;
		}
		else
		{
			Result.template tail<3>() = _Init.getInitialMagnetisationDirection();
		}

		afterStepCheck(Result); //normalize if necessary
		return Result;
	};
}
#endif //_BROWNANDNEELRELAXATION_H_

