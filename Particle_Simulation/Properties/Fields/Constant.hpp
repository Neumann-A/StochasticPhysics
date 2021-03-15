
#ifndef INC_ConstantFieldProperties_H
#define INC_ConstantFieldProperties_H
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
    struct Constant :General<prec> {

        using ThisClass = Constant<prec>;
        

        static const IField _TypeOfField{ Properties::IField::Field_Constant };

        typedef Eigen::Matrix<prec, 3, 1>        Vec3D;
        using Vec3DList                         = std::vector<Vec3D>;

        Vec3DList                                _Amplitudes{ Vec3D::Zero() };

        const Vec3DList& getAmplitudes() noexcept { return _Amplitudes; };
        inline void setAmplitudes(const Vec3DList& amplitudes) noexcept { _Amplitudes = amplitudes; };
    };

    template<typename Precision, typename Archive>
    void serialize(Constant<Precision>& val, Archive& ar)
    {
        /*std::string str{ to_string(val._TypeOfField) };
        ar(Archives::createNamedValue(std::string{ "Type_of_field" }, str));*/

        serializeVector(ar, "Number_of_Amplitudes", "Amplitude_", val._Amplitudes);
    }

}
#endif