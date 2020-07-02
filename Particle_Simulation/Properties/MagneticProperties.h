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

#include <type_traits>
#include <cstddef>
#include <exception>

#include <map>
#include <vector>
#include <algorithm>
#include <functional>

#include <Eigen/Core>

#include <MyCEL/math/Geometry.h>
#include <MyCEL/basics/BasicIncludes.h>

#include <SerAr/Core/NamedValue.h>

#include <Problems/Anisotropy/AnisotropyList.h>

///-------------------------------------------------------------------------------------------------
/// <signature>	Properties </signature>
///
/// <summary>	Namesapce for different Property classes. </summary>
///-------------------------------------------------------------------------------------------------

namespace Properties
{


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
		prec									SaturationMagnetisation{ 1.0 };
		prec									DampingConstant{ 1.0 };
		prec									GyromagneticRatio{ 1.0 };
		IAnisotropy								TypeOfAnisotropy{ IAnisotropy::Anisotropy_undefined };
		std::vector<prec>						AnisotropyConstants{ 0.0 };

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
			/*, NumberOfAnisotropyConstants(AnisotropyConstants.size())*/
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
		/// <summary>	Calculates the magnetic volume. </summary>
		///
		/// <returns>	The magnetic volume. </returns>
		///-------------------------------------------------------------------------------------------------
		inline prec getMagneticVolume() const noexcept { return math::geometry::sphere::calcVolume(getMagneticRadius()); };
		
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets saturation moment. </summary>
		///
		/// <returns>	The saturation moment. </returns>
		///-------------------------------------------------------------------------------------------------
		inline  prec getSaturationMoment() const noexcept { return (getMagneticVolume()*SaturationMagnetisation); };

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
		inline auto getNumberOfAnisotropyConstants() const noexcept { return (getAnisotropyConstants().size()); };

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
			//NumberOfAnisotropyConstants = values.size();
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

		ThisClass& operator+=(const ThisClass& rhs)						// compound assignment (does not need to be a member,
		{																// but often is, to modify the private members)
			if (TypeOfAnisotropy != rhs.getTypeOfAnisotropy())
			{
				throw std::runtime_error{ "Cannot add MagneticProperties due to different types of anisotropy!" };
			}
			if (AnisotropyConstants.size() != rhs.getAnisotropyConstants().size())
			{
				throw std::runtime_error{ "Cannot add MagneticProperties due to different numbers of anisotropy constants!" };
			}
			
			MagneticRadius += rhs.getMagneticRadius();
			SaturationMagnetisation += rhs.getSaturationMagnetisation();
			DampingConstant += rhs.getDampingConstant();
			GyromagneticRatio += rhs.getGyromagneticRatio();
			
			std::transform(AnisotropyConstants.begin(), AnisotropyConstants.end(), rhs.getAnisotropyConstants().cbegin(), AnisotropyConstants.begin(), std::plus<prec>());

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
			MagneticRadius /= (double)scalar;
			SaturationMagnetisation /= (double)scalar;
			DampingConstant /= (double)scalar;
			GyromagneticRatio /= (double)scalar;

			std::for_each(AnisotropyConstants.begin(), AnisotropyConstants.end(), [&scalar](auto& val) { val /= (double)scalar; });
			//std::transform(AnisotropyConstants.begin(), AnisotropyConstants.end(), AnisotropyConstants.begin(), std::divides<prec>(static_cast<prec>(scalar)));
			return *this;
		}

	};
}

#endif	// INC_MagneticProperties_H
// end of MagneticProperties.h
///---------------------------------------------------------------------------------------------------


