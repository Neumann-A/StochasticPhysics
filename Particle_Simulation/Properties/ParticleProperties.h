///---------------------------------------------------------------------------------------------------
// file:    ParticleProperties.h
//
// summary:     Declares the particle properties class
//
//            Copyright (c) 2016 Alexander Neumann.
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

#include <SerAr/Core/NamedValue.h>

namespace Properties
{
    ///-------------------------------------------------------------------------------------------------
    /// <summary>    The particles properties. </summary>
    ///
    /// <seealso cref="T:IConfigFileAll{ParticlesProperties{prec}}"/>
    /// <seealso cref="T:IMATLABFileWriteable{ParticlesProperties{prec}}"/>
    ///-------------------------------------------------------------------------------------------------
    template<typename prec>
    class ParticlesProperties
    {
    private:
        typedef ::Properties::ParticlesProperties<prec>                ThisClass;

    public:
        typedef typename Properties::MagneticProperties<prec>        MagneticProperties;
        typedef typename Properties::HydrodynamicProperties<prec>    HydrodynamicProperties;

    private: 
        //TODO: Move Temperature and Viscosity from ParticleProperties into some kind of new environment class. Per se those are not particle properties
        prec                          mTemperature{ 300.0 };
        /// <summary>    Magnetic Properties of the Particle. </summary>
        MagneticProperties            mMagProp{};
        /// <summary>    Hydrodynamic Properties of the Particle. </summary>
        HydrodynamicProperties        mHydroProp{};

    public:
        ///-------------------------------------------------------------------------------------------------
        /// <summary>    Particle properties. </summary>
        ///
        /// <param name="Mag">      The magnetic properties. </param>
        /// <param name="Hydro">    The hydrodynamic properties. </param>
        ///-------------------------------------------------------------------------------------------------
        constexpr inline ParticlesProperties(const prec &Temp, const MagneticProperties &Mag, const HydrodynamicProperties &Hydro) : mTemperature(Temp),mMagProp(Mag), mHydroProp(Hydro) {}
        inline ParticlesProperties() = default;

        // Access the Temperature
        inline const prec& getTemperature(void) const noexcept { return(mTemperature); }
        inline void setTemperature(const prec& temperature) noexcept { mTemperature = temperature; }

        ///-------------------------------------------------------------------------------------------------
        /// <summary>    Gets magnetic properties. </summary>
        ///
        /// <returns>    The magnetic properties. </returns>
        ///-------------------------------------------------------------------------------------------------
        inline const MagneticProperties& getMagneticProperties() const noexcept { return mMagProp; };
        inline MagneticProperties& modMagneticProperties() noexcept { return mMagProp; };

        ///-------------------------------------------------------------------------------------------------
        /// <summary>    Gets hydrodynamic properties. </summary>
        ///
        /// <returns>    The hydrodynamic properties. </returns>
        ///-------------------------------------------------------------------------------------------------
        inline const HydrodynamicProperties& getHydrodynamicProperties() const noexcept { return mHydroProp; };
        inline HydrodynamicProperties& modHydrodynamicProperties() noexcept { return mHydroProp; };

        ///-------------------------------------------------------------------------------------------------
        /// <summary>    Sets magnetic properties. </summary>
        ///
        /// <param name="Prop">    The magnetic properties. </param>
        ///-------------------------------------------------------------------------------------------------
        inline void setMagneticProperties(const MagneticProperties& Prop) noexcept { this->mMagProb = Prop; };

        ///-------------------------------------------------------------------------------------------------
        /// <summary>    Sets hydrodynamic properties. </summary>
        ///
        /// <param name="Prop">    The hydrodynamic properties. </param>
        ///-------------------------------------------------------------------------------------------------
        inline void setHydrodynamicProperties(const HydrodynamicProperties& Prop) noexcept { this->mHydroProb = Prop; };

        static inline std::string getSectionName() noexcept { return std::string{ "Particle_Properties" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Temperature", mTemperature));
            ar(Archives::createNamedValue(MagneticProperties::getSectionName(), mMagProp));
            ar(Archives::createNamedValue(HydrodynamicProperties::getSectionName(), mHydroProp));
        }
        
        ThisClass& operator+=(const ThisClass& rhs)                        // compound assignment (does not need to be a member,
        {                                                                // but often is, to modify the private members)
            mTemperature += rhs.getTemperature();
            mMagProp += rhs.getMagneticProperties();
            mHydroProp += rhs.getHydrodynamicProperties();
            return *this;                                                // return the result by reference
        }

        // friends defined inside class body are inline and are hidden from non-ADL lookup
        friend ThisClass operator+(ThisClass lhs, const ThisClass& rhs) // passing lhs by value helps optimize chained a+b+c
        {                                                                // otherwise, both parameters may be const references
            lhs += rhs;                                                    // reuse compound assignment
            return lhs;                                                    // return the result by value (uses move constructor)
        }

        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value,ThisClass&> operator/=(const Number& scalar)
        {
            mTemperature /= (double)scalar;
            mMagProp /= (double)scalar;
            mHydroProp /= (double)scalar;
            return *this;
        }
    };

}

#endif    // INC_ParticleProperties_H
// end of ParticleProperties.h
///---------------------------------------------------------------------------------------------------
