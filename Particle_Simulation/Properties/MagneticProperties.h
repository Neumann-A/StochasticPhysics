///---------------------------------------------------------------------------------------------------
// file:	MagneticProperties.h
//
// summary: 	Declares the class for magnetic properties 
//
//			Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 05.06.2016

#ifndef INC_MagneticProperties_H
#define INC_MagneticProperties_H
///---------------------------------------------------------------------------------------------------

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#define M_PI       3.14159265358979323846

#include <map>
#include <vector>

#include <Eigen/Core>

#include "basics/Logger.h"
#include "basics/BasicIncludes.h"

#include "Archive/NamedValue.h"
//Forward Declare all Anisotropies
template <typename prec>
class UniaxialAnisotropy;

///-------------------------------------------------------------------------------------------------
/// <signature>	Properties </signature>
///
/// <summary>	Namesapce for different Property classes. </summary>
///-------------------------------------------------------------------------------------------------

namespace Properties
{

	/// <summary>	Values that represent anisotropies. </summary>
	enum class IAnisotropy { Anisotropy_undefined, Anisotropy_uniaxial };

	template <IAnisotropy Anisotropy>
	class AnisotropyTypeSelector;

	template <>
	class AnisotropyTypeSelector<IAnisotropy::Anisotropy_uniaxial>
	{
	public:
		template<typename prec>
		using type = UniaxialAnisotropy<prec>;
	};

#ifdef _MSC_VER
#pragma warning (push)
#pragma warning ( disable : 4592) // Disable stupid VS Debug message
#endif
	/// <summary>	Map used to change the IAnisotropy enum to a string and vice versa. </summary>
	static const std::map<IAnisotropy, std::string> IAnisotropyMap{ { { IAnisotropy::Anisotropy_undefined,"undefined" },{ IAnisotropy::Anisotropy_uniaxial,"uniaxial" } } };
#ifdef _MSC_VER	
#pragma warning (pop)
#endif

	template<typename T>
	T from_string(const std::string&);

	///-------------------------------------------------------------------------------------------------
	/// <summary>	Gets the enum IAnisotropy from a string. </summary>
	///
	/// <param name="AnisoString">	The string to transform </param>
	///
	/// <returns>	An Enum representing the string  </returns>
	///-------------------------------------------------------------------------------------------------
	template<>
	IAnisotropy from_string<IAnisotropy>(const std::string &AnisoString);
	std::string to_string(const IAnisotropy& field);

	///-------------------------------------------------------------------------------------------------
	/// <summary>	Class which defines the Magnetic Properties of a Particle. </summary>
	///
	/// <seealso cref="T:IConfigFileAll{MagneticProperties{prec}}"/>
	/// <seealso cref="T:IMATLABFileWriteable{MagneticProperties{prec}}"/>
	///-------------------------------------------------------------------------------------------------
	template<typename prec>
	class MagneticProperties
	{
	private:
		typedef MagneticProperties<prec>		ThisClass;
		typedef Eigen::Matrix<prec, 3, 1>		Vec3D;


		prec									MagneticRadius{ 1E-9 };
		prec									SaturationMagnetisation{ 1 };
		prec									DampingConstant{ 1 };
		prec									GyromagneticRatio{ 1 };
		IAnisotropy								TypeOfAnisotropy{ IAnisotropy::Anisotropy_undefined };
		std::vector<prec>						AnisotropyConstants{ 0 };
		unsigned long long						NumberOfAnisotropyConstants{ 1 };

	protected:
	public:
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Magnetic properties. </summary>
		///
		/// <param name="MagRadius">	 	The Magnetic Radius. </param>
		/// <param name="SatMag">		 	The Saturation Magnetisation. </param>
		/// <param name="DampConst">	 	The Damping constant alpha. </param>
		/// <param name="GyromagRatio">  	The Gyromagnetic Ratio. </param>
		/// <param name="TypeOfAniso">   	The Type of Anisotropy. </param>
		/// <param name="AnisoConstants">	The Anisotropy Constants. </param>
		///-------------------------------------------------------------------------------------------------
		constexpr inline MagneticProperties(prec MagRadius, prec SatMag, prec DampConst,
			prec GyromagRatio, IAnisotropy TypeOfAniso, std::vector<prec> AnisoConstants)
			: MagneticRadius(MagRadius), SaturationMagnetisation(SatMag), DampingConstant(DampConst),
			GyromagneticRatio(GyromagRatio), TypeOfAnisotropy(TypeOfAniso), AnisotropyConstants(AnisoConstants)
			, NumberOfAnisotropyConstants(AnisotropyConstants.size())
		{};
		constexpr inline MagneticProperties() = default;
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets magnetic radius. </summary>
		///
		/// <returns>	The magnetic radius. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const prec& getMagneticRadius() const noexcept { return (MagneticRadius); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Sets magnetic radius. </summary>
		///
		/// <param name="value">	The new magnetic radius </param>
		///-------------------------------------------------------------------------------------------------
		inline void setMagneticRadius(const prec &value) noexcept { MagneticRadius = value; };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets Calculates the magnetic volume. </summary>
		///
		/// <returns>	The magnetic volume. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const prec getMagneticVolume() const noexcept { return (4 / 3 * M_PI*pow(getMagneticRadius(), 3)); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets saturation moment. </summary>
		///
		/// <returns>	The saturation moment. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const prec getSaturationMoment() const noexcept { return (getMagneticVolume()*SaturationMagnetisation); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets saturation magnetisation. </summary>
		///
		/// <returns>	The saturation magnetisation. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const prec& getSaturationMagnetisation() const noexcept { return (SaturationMagnetisation); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the damping constant alpha. </summary>
		///
		/// <returns>	The damping constant. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const prec& getDampingConstant() const noexcept { return (DampingConstant); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the gyromagnetic ratio. </summary>
		///
		/// <returns>	The gyromagnetic ratio. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const prec& getGyromagneticRatio() const noexcept { return (GyromagneticRatio); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the type of anisotropy. </summary>
		///
		/// <returns>	The type of anisotropy. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const IAnisotropy& getTypeOfAnisotropy() const noexcept { return (TypeOfAnisotropy); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the number of anisotropy constants. </summary>
		///
		/// <returns>	The number of anisotropy constants. </returns>
		///-------------------------------------------------------------------------------------------------
		inline unsigned long long getNumberOfAnisotropyConstants() const noexcept { return (NumberOfAnisotropyConstants); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the anisotropy constants. </summary>
		///
		/// <returns>	The anisotropy constants. </returns>
		///-------------------------------------------------------------------------------------------------
		inline const std::vector<prec>& getAnisotropyConstants() const noexcept { return (AnisotropyConstants); };

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Sets the anisotropy constants. </summary>
		///
		/// <param name="values">	The new anisotropy constants. </param>
		///-------------------------------------------------------------------------------------------------
		inline void setAnisotropyConstants(const std::vector<prec> &values) noexcept 
		{ 
			AnisotropyConstants = std::vector<prec>{ values.begin(), values.end() };
			NumberOfAnisotropyConstants = values.size();
		};

		static inline std::string getSectionName() { return std::string{ "Magnetic_Properties" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue("Magnetic_radius", MagneticRadius));
			ar(Archives::createNamedValue("Saturation_magnetisation", SaturationMagnetisation));
			ar(Archives::createNamedValue("Damping_constant", DampingConstant));
			ar(Archives::createNamedValue("Gyromagnetic_ratio", GyromagneticRatio));

			std::string str{ to_string(TypeOfAnisotropy) };
			ar(Archives::createNamedValue(std::string{ "Type_of_anisotropy" }, str));
			TypeOfAnisotropy = from_string<decltype(TypeOfAnisotropy)>(str);

			std::size_t NoAnisotropy = AnisotropyConstants.size();
			ar(Archives::createNamedValue("Number_of_anisotropies", NoAnisotropy));
			AnisotropyConstants.resize(NoAnisotropy);

			std::size_t counter{ 0 };
			for (auto& it : AnisotropyConstants)
			{
				ar(Archives::createNamedValue(std::string{ "Anisotropy_" } +BasicTools::toStringScientific(++counter), it));
			}
		}

	};
}

#endif	// INC_MagneticProperties_H
// end of MagneticProperties.h
///---------------------------------------------------------------------------------------------------


