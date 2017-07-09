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

#include <map>
#include <vector>

#include "math/Geometry.h"

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
		prec HydrodynamicRadius{ 0.0 };
		/// <summary>	the viscosity. </summary>
		prec Viscosity{ 0.0 };

	public:
		inline constexpr explicit HydrodynamicProperties(prec HydroRadius, prec viscosity)	: HydrodynamicRadius(HydroRadius), Viscosity(viscosity) {};
		inline constexpr explicit HydrodynamicProperties() = default;

		inline const prec getHydrodynamicVolume() const noexcept { return math::geometry::sphere::calcVolumeSphere(getHydrodynamicRadius()); };
		inline const prec& getHydrodynamicRadius() const noexcept { return HydrodynamicRadius; };
		inline void setHydrodynamicRadius(const prec &value) noexcept { HydrodynamicRadius = value; };
		inline const prec& getViscosity() const noexcept { return Viscosity; };

		static inline std::string getSectionName() { return std::string{ "Hydrodynamic_Properties" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue("Hydrodynamic_Radius", HydrodynamicRadius));
			ar(Archives::createNamedValue("Viscosity", Viscosity)); //TODO: Move this to more general class? (Because it is not really a particle properties but more of properties of the solution; just like temperature!)
		}

		ThisClass& operator+=(const ThisClass& rhs)						// compound assignment (does not need to be a member,
		{																// but often is, to modify the private members)
			HydrodynamicRadius += rhs.getHydrodynamicRadius();
			Viscosity += rhs.getViscosity();

			return *this;												// return the result by reference
		}

		// friends defined inside class body are inline and are hidden from non-ADL lookup
		friend ThisClass operator+(ThisClass lhs, const ThisClass& rhs) // passing lhs by value helps optimize chained a+b+c
		{															    // otherwise, both parameters may be const references
			lhs += rhs;													// reuse compound assignment
			return lhs;													// return the result by value (uses move constructor)
		}

		template<typename Number>
		std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
		{
			HydrodynamicRadius /= scalar;
			Viscosity /= scalar;
			return *this;
		}
		
	};
}

#endif	// INC_HydrodynamicProperties_H
// end of HydrodynamicProperties.h
///---------------------------------------------------------------------------------------------------
