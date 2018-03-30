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

//#include <string>

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
	//TODO: Find a more maintainable and extensible solution for this enum 
	//		which can also be used in templates! (Solver, Problem, Field)
	enum class IField { Field_undefined, Field_Zero, Field_Constant, Field_Sinusoidal, Field_Lissajous, Field_Triangular};
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning ( disable : 4592) // Disable VS Debug message
#endif
	static const std::map<IField, std::string> IFieldMap{ { { IField::Field_undefined,"undefined" },
															{ IField::Field_Zero,"none" },
															{ IField::Field_Constant,"constant" },
															{ IField::Field_Sinusoidal,"sinusoidal" },
															{ IField::Field_Lissajous,"lissajous" },
															{ IField::Field_Triangular,"triangular"}} };
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

		std::vector<prec>						_FrequenciesPeriodes{ 0 };
		std::vector<prec>						_PhasesTimeOffsets{ 0 };
		std::vector<prec>						_Periodes{ 0 };

		static inline std::string buildSerilizationString(const char* name, const std::size_t& number)
		{
			return std::string{ name + BasicTools::toStringScientific(number) };
		}
		
		template<typename Archive, typename Container>
		static inline void serializeVector(Archive &ar, const char* sizevector, const char* vecname, Container& vector)
		{
			auto elements = vector.size();
			ar(Archives::createNamedValue(sizevector, elements));
			vector.resize(elements);

			std::size_t counter{ 0 };
			for (auto& it : vector)
			{
				ar(Archives::createNamedValue(buildSerilizationString(vecname,++counter), it));
			}
		}
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		explicit FieldProperties(const IField& field, const Vec3DList& amplitudes, const std::vector<prec>& freqorperiods, const std::vector<prec>& phases)
			: _TypeOfField(field), _Amplitudes(amplitudes), _FrequenciesPeriodes(freqorperiods), _PhasesTimeOffsets(phases)	{};
		FieldProperties() {};

		const IField& getTypeOfField() const noexcept { return _TypeOfField; };
		const Vec3DList& getAmplitudes() const noexcept { return _Amplitudes; };
		Vec3DList& getAmplitudes() noexcept { return _Amplitudes; };
		const std::vector<prec>& getFrequencies() const noexcept { return _FrequenciesPeriodes; };
		const std::vector<prec>& getPhases() const noexcept { return _PhasesTimeOffsets; };
		const std::vector<prec>& getPeriodes() const noexcept { return _FrequenciesPeriodes; };
		const std::vector<prec>& getTimeOffsets() const noexcept { return _PhasesTimeOffsets; };

		inline void setAmplitudes(const Vec3DList& amplitudes) noexcept { _Amplitudes = amplitudes; };

		static std::string getSectionName() noexcept { return std::string{ "Field_Properties" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			std::string str{to_string(_TypeOfField)};
			ar(Archives::createNamedValue(std::string{ "Type_of_field" }, str));
			_TypeOfField = from_string<decltype(_TypeOfField)>(str);
	
			serializeVector(ar, "Number_of_Amplitudes","Amplitude_", _Amplitudes);

			switch (_TypeOfField)
			{
			case IField::Field_Triangular:
				serializeVector(ar, "Number_of_Periodes","Periode_", _FrequenciesPeriodes);
				serializeVector(ar, "Number_of_Timeoffsets","Timeoffset_", _PhasesTimeOffsets);

				
				break;
			case IField::Field_Sinusoidal:
			case IField::Field_Lissajous:
				serializeVector(ar, "Number_of_Frequencies", "Frequency_", _FrequenciesPeriodes);
				serializeVector(ar, "Number_of_Phases", "Phase_", _PhasesTimeOffsets);
				break;
			default:
				break;
			}
			


			

		}
	};
}

#endif	// INC_FieldProperties_H
// end of FieldProperties.h
///---------------------------------------------------------------------------------------------------
