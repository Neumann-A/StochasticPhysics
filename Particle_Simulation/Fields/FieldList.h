///---------------------------------------------------------------------------------------------------
// file:		Fields\FieldList.h
//
// summary: 	Declares the anisotropy list class
//
// Copyright (c) 2018 Alexander Neumann.
//
// author: Alexander
// date: 14.06.2018

#ifndef INC_FieldList_H
#define INC_FieldList_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <string_view>
#include <string>

#include <MyCEL/types/static_map.hpp>

//Forward Declare all Fields

    template <typename prec>
    class ZeroField;
    template <typename prec>
    class ConstantField;
    template <typename prec>
    class LissajousField;
    template <typename prec>
    class RectangularField;
    template <typename prec>
    class SinusoidalField;
    template <typename prec>
    class TriangularField;
    template <typename prec>
    class SincField;

    template <typename Field>
    class FieldTraits;

namespace Properties
{
    namespace
    {
        using namespace std::literals::string_view_literals;
    }
    /// <summary>	Values that represent the used fields. </summary>
    enum class IField {Field_Zero=1,Field_Constant, Field_Sinusoidal, Field_Lissajous,Field_Triangular, Field_Rectangular,Field_Sinc};

    /// <summary>	Map used to change the IField enum to a string and vice versa. </summary>
    constexpr MyCEL::static_map<IField, std::string_view, 7> IFieldMap { { { 
                            { IField::Field_Zero, "none"sv },
                            { IField::Field_Constant, "constant"sv },
                            { IField::Field_Sinusoidal, "sinusoidal"sv },
                            { IField::Field_Lissajous, "lissajous"sv },
                            { IField::Field_Triangular,"triangular"sv },
                            { IField::Field_Rectangular,"rectangular"sv },
                            { IField::Field_Sinc,"sinc"sv }
                             } }};
    constexpr auto IFieldValues{ IFieldMap.get_key_array() };

    template<typename T>
    T from_string(const std::string&);

    template<IField value>
    struct FieldSelector;

    ///-------------------------------------------------------------------------------------------------
    /// <summary>	Gets the enum IField from a string. </summary>
    ///
    /// <param name="FieldString">	The string to transform </param>
    ///
    /// <returns>	An Enum representing the string  </returns>
    ///-------------------------------------------------------------------------------------------------
    template<>
    IField from_string<IField>(const std::string& FieldString);
    std::string to_string(const IField& field);

    IField from_string(std::string_view, IField&);

    std::string to_string(const IField& field);
}
#endif	// INC_FieldList_H
// end of Fields\FieldList.h
