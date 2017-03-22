///---------------------------------------------------------------------------------------------------
// file:		HydrodynamicProperties.h
//
// summary: 	Declares the hydrodynamic properties class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 25.06.2016

#ifndef INC_HydrodynamicProperties_H
#define INC_HydrodynamicProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#define M_PI       3.14159265358979323846

#include <map>
#include <vector>

#include "Archive/NamedValue.h"

namespace Properties
{
	///-------------------------------------------------------------------------------------------------
	/// <summary>	Class describing hydrodynamic properties. </summary>
	///
	/// <seealso cref="T:IConfigFileAll{HydrodynamicProperties{prec}}"/>
	/// <seealso cref="T:IMATLABFileWriteable{HydrodynamicProperties{prec}}"/>
	///-------------------------------------------------------------------------------------------------
	template<typename prec>
	class HydrodynamicProperties
	{
	private:
		typedef HydrodynamicProperties<prec> ThisClass;

		/// <summary>	the hydrodynamic radius. </summary>
		prec HydrodynamicRadius{ 0 };
		/// <summary>	the viscosity. </summary>
		prec Viscosity{ 0 };

	public:
		inline constexpr explicit HydrodynamicProperties(prec HydroRadius, prec viscosity)	: HydrodynamicRadius(HydroRadius), Viscosity(viscosity) {};
		inline constexpr explicit HydrodynamicProperties() = default;

		inline const prec getHydrodynamicVolume() const noexcept { return (4 / 3 * M_PI*pow(HydrodynamicRadius, 3)); };
		inline const prec& getHydrodynamicRadius() const noexcept { return HydrodynamicRadius; };
		inline void setHydrodynamicRadius(const prec &value) noexcept { HydrodynamicRadius = value; };
		inline const prec& getViscosity() const noexcept { return Viscosity; };

		static inline std::string getSectionName() { return std::string{ "Hydrodynamic_Properties" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue("Hydrodynamic_Radius", HydrodynamicRadius));
			ar(Archives::createNamedValue("Viscosity", Viscosity)); //TODO: Move this to more general case?
		}
	};
}

#endif	// INC_HydrodynamicProperties_H
// end of HydrodynamicProperties.h
///---------------------------------------------------------------------------------------------------
