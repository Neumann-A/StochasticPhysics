
#ifndef INC_RectangularFieldProperties_H
#define INC_RectangularFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <string>
#include <map>

#include <Eigen/Core>
//#include <Eigen/StdVector>

#include <SerAr/Core/NamedValue.h>
#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/OutputArchive.h>

#include <MyCEL/basics/BasicIncludes.h>
#include "Fields/FieldList.h"
#include "Field.hpp"
namespace Properties::Fields
{
    template<typename prec>
    struct Rectangular :General<prec>{

        typedef Eigen::Matrix<prec, 3, 1>                                Vec3D;
        using Vec3DList = std::vector<Vec3D>;
        static const IField          _TypeOfField{ Properties::IField::Field_Rectangular };

        Vec3DList                    _Amplitudes{ Vec3D::Zero() };
        std::vector<prec>            _Periodes{ 0 };
        std::vector<prec>            _PhasesTimeOffsets{ 0 };
        prec                         _Tau{ 0.0 };
        bool                         _alternating{ false };

        using ThisClass = Rectangular<prec>;

        const Vec3DList& getAmplitudes() noexcept { return _Amplitudes; };
        inline void setAmplitudes(const Vec3DList& amplitudes) noexcept { _Amplitudes = amplitudes; };
    };

    template<typename Precision, typename Archive>
    void serialize(Rectangular<Precision>& val, Archive& ar)
    {
       /* std::string str{ to_string(val._TypeOfField) };
        ar(Archives::createNamedValue(std::string{ "Type_of_field" }, str));*/

        serializeVector(ar, "Number_of_Amplitudes", "Amplitude_", val._Amplitudes);
        serializeVector(ar, "Number_of_Periodes", "Periode_", val._Periodes);
        serializeVector(ar, "Number_of_Phases", "Phase_", val._PhasesTimeOffsets);
        ar(Archives::createNamedValue("Tau", val._Tau));
        ar(Archives::createNamedValue("Alternating", val._alternating));
    }
}
#endif