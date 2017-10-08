///---------------------------------------------------------------------------------------------------
// file:		FieldProperties.h
//
// summary: 	Declares the field properties class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 20.06.2016

#ifndef INC_FieldProperties_H
#define INC_FieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Archive/NamedValue.h"
#include "Archive/InputArchive.h"
#include "Archive/OutputArchive.h"

#include "basics/BasicIncludes.h"

template<typename precision>
class SinusoidalField;

template <typename prec>
class LissajousField;

namespace Properties
{
	enum class IField { Field_undefined, Field_Zero, Field_Constant, Field_Sinusoidal, Field_Lissajous };
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning ( disable : 4592) // Disable VS Debug message
#endif
	static const std::map<IField, std::string> IFieldMap{ { { IField::Field_undefined,"undefined" },{ IField::Field_Zero,"none" },{ IField::Field_Constant,"constant" },{ IField::Field_Sinusoidal,"sinusoidal" },{ IField::Field_Lissajous,"lissajous" } } };
#ifdef _MSC_VER
#pragma warning (pop)
#endif

	template<typename T>
	T from_string(const std::string&);

	template<>
	IField from_string<IField>(const std::string&);

	std::string to_string(const IField& field);
	
	template <typename prec>
	class FieldProperties
	{
	private:
		typedef FieldProperties<prec>									ThisClass;
		typedef Eigen::Matrix<prec, 3, 1>								Vec3D;
		typedef std::vector<Vec3D, Eigen::aligned_allocator<Vec3D>>		Vec3DList;
	public:
		typedef prec											Precision;

	private:
		IField									_TypeOfField{ IField::Field_undefined };		
		Vec3DList								_Amplitudes{Vec3D::Zero()};
		std::vector<prec>						_Frequencies{ 0 };
		std::vector<prec>						_Phases{ 0 };

		static inline std::string BuildAmplitudeString(const std::size_t& number)
		{
			return std::string{ "Amplitude_" + BasicTools::toStringScientific(number) };
		}

		static inline std::string BuildFrequencyString(const std::size_t& number)
		{
			return std::string{ "Frequency_" + BasicTools::toStringScientific(number) };
		}

		static inline std::string BuildPhaseString(const std::size_t& number)
		{
			return std::string{ "Phase_" + BasicTools::toStringScientific(number) };
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		explicit FieldProperties(const IField& field, const Vec3DList& amplitudes, const std::vector<prec>& frequencies, const std::vector<prec>& phases)
			: _TypeOfField(field), _Amplitudes(amplitudes), _Frequencies(frequencies), _Phases(phases)	{};
		FieldProperties() {};

		const IField& getTypeOfField() const noexcept { return _TypeOfField; };
		const Vec3DList& getAmplitudes() const noexcept { return _Amplitudes; };
		Vec3DList& getAmplitudes() noexcept { return _Amplitudes; };
		const std::vector<prec>& getFrequencies() const noexcept { return _Frequencies; };
		const std::vector<prec>& getPhases() const noexcept { return _Phases; };

		inline void setAmplitudes(const Vec3DList& amplitudes) noexcept { _Amplitudes = amplitudes; };

		static std::string getSectionName() noexcept { return std::string{ "Field_Properties" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			auto NoAmplitudes{ _Amplitudes.size() };
			auto NoFrequencies{ _Frequencies.size() };
			auto NoPhases{ _Phases.size() };

			std::string str{to_string(_TypeOfField)};
			ar(Archives::createNamedValue(std::string{ "Type_of_field" }, str));
			_TypeOfField = from_string<decltype(_TypeOfField)>(str);

			ar(Archives::createNamedValue("Number_of_Amplitudes", NoAmplitudes) );
			ar(Archives::createNamedValue("Number_of_Frequencies", NoFrequencies) );
			ar(Archives::createNamedValue("Number_of_Phases", NoPhases) );

			_Amplitudes.resize(NoAmplitudes);
			_Frequencies.resize(NoFrequencies);
			_Phases.resize(NoPhases);

			std::size_t counter{ 0 };
			for (auto& it : _Amplitudes)
			{
				ar(Archives::createNamedValue(BuildAmplitudeString(++counter), it));
			}
			
			counter = 0;
			for (auto& it : _Frequencies)
			{
				ar(Archives::createNamedValue(BuildFrequencyString(++counter), it));
			}
			
			counter = 0;
			for (auto& it : _Phases)
			{
				ar(Archives::createNamedValue(BuildPhaseString(++counter), it));
			}
		}
	};
}

#endif	// INC_FieldProperties_H
// end of FieldProperties.h
///---------------------------------------------------------------------------------------------------
