///---------------------------------------------------------------------------------------------------
// file:	ParticleProperties.h
//
// summary: 	Declares the particle properties class
//
//			Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 05.06.2016

#ifndef INC_ParticleProperties_H
#define INC_ParticleProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>

//New Parameter Set
#include "MagneticProperties.h"
#include "HydrodynamicProperties.h"

#include "Archive/NamedValue.h"

namespace Properties
{
	///-------------------------------------------------------------------------------------------------
	/// <summary>	The particles properties. </summary>
	///
	/// <seealso cref="T:IConfigFileAll{ParticlesProperties{prec}}"/>
	/// <seealso cref="T:IMATLABFileWriteable{ParticlesProperties{prec}}"/>
	///-------------------------------------------------------------------------------------------------
	template<typename prec>
	class ParticlesProperties
	{
	private:
		typedef ::Properties::ParticlesProperties<prec>				ThisClass;

	public:
		typedef typename Properties::MagneticProperties<prec>		MagneticProperties;
		typedef typename Properties::HydrodynamicProperties<prec>	HydrodynamicProperties;

	private: 
		//TODO: Move Temperature and Viscosity from ParticleProperties into some kind of new environment class. Per se those are not particle properties
		prec						_Temperature{ 300.0 };
		/// <summary>	Magnetic Properties of the Particle. </summary>
		MagneticProperties			_MagProp{};
		/// <summary>	Hydrodynamic Properties of the Particle. </summary>
		HydrodynamicProperties		_HydroProp{};

	public:
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Particle properties. </summary>
		///
		/// <param name="Mag">  	The magnetic properties. </param>
		/// <param name="Hydro">	The hydrodynamic properties. </param>
		///-------------------------------------------------------------------------------------------------
		constexpr inline ParticlesProperties(const prec &Temp, const MagneticProperties &Mag, const HydrodynamicProperties &Hydro) : _Temperature(Temp),_MagProp(Mag), _HydroProp(Hydro) {};
		inline ParticlesProperties() = default;

		// Access the Temperature
		inline const prec& getTemperature(void) const noexcept { return(_Temperature); }
		inline void setTemperature(const prec& temperature) noexcept { _Temperature = temperature; }

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets magnetic properties. </summary>
		///
		/// <returns>	The magnetic properties. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const MagneticProperties& getMagneticProperties() const noexcept { return _MagProp; };
		inline MagneticProperties& modMagneticProperties() noexcept { return _MagProp; };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets hydrodynamic properties. </summary>
		///
		/// <returns>	The hydrodynamic properties. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const HydrodynamicProperties& getHydrodynamicProperties() const noexcept { return _HydroProp; };
		inline HydrodynamicProperties& modHydrodynamicProperties() noexcept { return _HydroProp; };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Sets magnetic properties. </summary>
		///
		/// <param name="Prop">	The magnetic properties. </param>
		///-------------------------------------------------------------------------------------------------
		inline void setMagneticProperties(const MagneticProperties& Prop) noexcept { this->_MagProb = Prop; };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Sets hydrodynamic properties. </summary>
		///
		/// <param name="Prop">	The hydrodynamic properties. </param>
		///-------------------------------------------------------------------------------------------------
		inline void setHydrodynamicProperties(const HydrodynamicProperties& Prop) noexcept { this->_HydroProb = Prop; };

		static inline std::string getSectionName() noexcept { return std::string{ "Particle_Properties" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue("Temperature", _Temperature));
			ar(Archives::createNamedValue(MagneticProperties::getSectionName(), _MagProp));
			ar(Archives::createNamedValue(HydrodynamicProperties::getSectionName(), _HydroProp));
		}
		
		ThisClass& operator+=(const ThisClass& rhs)						// compound assignment (does not need to be a member,
		{																// but often is, to modify the private members)
			_Temperature += rhs.getTemperature();
			_MagProp += rhs.getMagneticProperties();
			_HydroProp += rhs.getHydrodynamicProperties();
			return *this;												// return the result by reference
		}

		// friends defined inside class body are inline and are hidden from non-ADL lookup
		friend ThisClass operator+(ThisClass lhs, const ThisClass& rhs) // passing lhs by value helps optimize chained a+b+c
		{															    // otherwise, both parameters may be const references
			lhs += rhs;													// reuse compound assignment
			return lhs;													// return the result by value (uses move constructor)
		}

		template<typename Number>
		std::enable_if_t<std::is_arithmetic<Number>::value,ThisClass&> operator/=(const Number& scalar)
		{
			_Temperature /= scalar;
			_MagProp /= scalar;
			_HydroProp /= scalar;
			return *this;
		}
	};

};

#endif	// INC_ParticleProperties_H
// end of ParticleProperties.h
///---------------------------------------------------------------------------------------------------
