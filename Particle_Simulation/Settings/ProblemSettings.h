///---------------------------------------------------------------------------------------------------
// file:		ProblemSettings.h
//
// summary: 	Declares the problem settings class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 24.06.2016

#ifndef INC_ProblemSettings_H
#define INC_ProblemSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <memory>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <string_view>

#include <MyCEL/types/static_map.hpp>
#include <MyCEL/basics/enumhelpers.h>
#include <MyCEL/basics/templatehelpers.h>

#include <SerAr/Core/NamedValue.h>
#include <SerAr/Core/LoadConstructor.h>
#include <SerAr/Core/OutputArchive.h>
#include <SerAr/Core/InputArchive.h>

namespace Settings
{
    //Forward declrations of problem settings:
    template<typename prec>
    class NoProblemSettings {
        template<typename Archive>
        void serialize(Archive &ar) {}
    };

    template<typename prec>
    class NeelProblemSettings;
    template<typename prec>
    class NeelSphericalProblemSettings;
    template <typename prec>
    class BrownAndNeelProblemSettings;
    template <typename prec>
    class BrownAndNeelEulerSphericalProblemSettings;


    //TODO: Find a more maintainable and extensible solution for this enum 
    //		which can also be used in templates! (Solver, Problem, Field)
    /// <summary>	Values that represent differen Problems to simulate. </summary>
    enum class IProblem {   Problem_undefined, 
                            Problem_Neel,
                            Problem_NeelSpherical, 
                            Problem_BrownAndNeel, 
                            Problem_BrownAndNeelEulerSpherical

                            /* Problem_NeelQuaternion	*/};

    inline constexpr const MyCEL::static_map<IProblem, std::string_view, 5> IProblemMap { { { 
                            { IProblem::Problem_undefined,"undefined" },
                            { IProblem::Problem_Neel, "Neel"},
                            { IProblem::Problem_NeelSpherical, "NeelSpherical"},
                            { IProblem::Problem_BrownAndNeel,"BrownAndNeel" },
                            { IProblem::Problem_BrownAndNeelEulerSpherical,"BrownAndNeelEulerSpherical" }
                            // { IProblem::Problem_NeelQuaternion,"NeelQuaternion" }
                            } } };
    inline constexpr const auto IProblemValues{ IProblemMap.get_key_array() };
    namespace {
        constexpr auto getValidIProblemValues() { 
            std::array<IProblem, IProblemValues.size()-1> ret;
            std::copy(begin(IProblemValues)+1,end(IProblemValues), begin(ret));
            return ret;
        }
    }
    inline constexpr const auto ValidIProblemValues {getValidIProblemValues()};
    std::string to_string(const IProblem&);
    template<typename T>
    T from_string(const std::string&);
    template<>
    IProblem from_string<IProblem>(const std::string&);
    IProblem from_string(std::string_view str, IProblem& value);

    inline std::string getTypeNameDescription(IProblem) {
        return "Type_of_problem";
    }

    template<IProblem value>
    struct ProblemSettingsSelector;

    template<>
    struct ProblemSettingsSelector<IProblem::Problem_BrownAndNeel> 
        : MyCEL::enum_value_type<IProblem, IProblem::Problem_BrownAndNeel> {
        template<typename prec>
        using type = BrownAndNeelProblemSettings<prec>;
    };
    template<>
    struct ProblemSettingsSelector<IProblem::Problem_BrownAndNeelEulerSpherical> 
        : MyCEL::enum_value_type<IProblem, IProblem::Problem_BrownAndNeelEulerSpherical> {
        template<typename prec>
        using type = BrownAndNeelEulerSphericalProblemSettings<prec>;
    };
    template<>
    struct ProblemSettingsSelector<IProblem::Problem_Neel> 
        : MyCEL::enum_value_type<IProblem, IProblem::Problem_Neel> {
        template<typename prec>
        using type = NeelProblemSettings<prec>;
    };
    template<>
    struct ProblemSettingsSelector<IProblem::Problem_NeelSpherical> 
        : MyCEL::enum_value_type<IProblem, IProblem::Problem_NeelSpherical> {
        template<typename prec>
        using type = NeelSphericalProblemSettings<prec>;
    };

    template<typename prec>
    class BrownAndNeelProblemSettings
    {
    public:
        bool _UseSimpleModel{false};
        static std::string getSectionName() { return std::string{ "BrownAndNeel_Problem_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Use_simple_model", _UseSimpleModel));
        }

        // Access the UseSimpleModel
        bool getUseSimpleModel(void) const noexcept { return(_UseSimpleModel); }
        void setUseSimpleModel(bool useSimpleModel)	noexcept { _UseSimpleModel = useSimpleModel; }
    };
    template<typename prec>
    inline std::string getTypeNameDescription(const BrownAndNeelProblemSettings<prec>&)
    {
        return BrownAndNeelProblemSettings<prec>::getSectionName();
    }

    template<typename prec>
    class BrownAndNeelEulerSphericalProblemSettings
    {
    public:
        bool mUseSphericalCoordinateTransformation{ false };
        bool mUseEulerCoordinateTransformation{ false };
        prec mNeelMinAngleBeforeTransformation{ std::numeric_limits<prec>::epsilon() };
        prec mBrownMinAngleBeforeTransformation{ std::numeric_limits<prec>::epsilon() };

        static std::string getSectionName() { return std::string{ "BrownAndNeelEulerSpherical_Problem_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Use_spherical_coordinate_transformation", mUseSphericalCoordinateTransformation));
            ar(Archives::createNamedValue("Use_euler_coordinate_transformation", mUseEulerCoordinateTransformation));
            ar(Archives::createNamedValue("Neel_Min_angle_before_transformation", mNeelMinAngleBeforeTransformation));
            ar(Archives::createNamedValue("Brown_Min_angle_before_transformation", mBrownMinAngleBeforeTransformation));

            assert(mNeelMinAngleBeforeTransformation >= (prec)0.0);
            assert(mNeelMinAngleBeforeTransformation <= (prec)std::acos(-1));
            assert(mBrownMinAngleBeforeTransformation >= (prec)0.0);
            assert(mBrownMinAngleBeforeTransformation <= (prec)std::acos(-1));
        }
    };
    template<typename prec>
    inline std::string getTypeNameDescription(const BrownAndNeelEulerSphericalProblemSettings<prec>&)
    {
        return BrownAndNeelEulerSphericalProblemSettings<prec>::getSectionName();
    }

    template<typename prec>
    class NeelProblemSettings
    {
    public:
        static std::string getSectionName() { return std::string{ "Neel_Problem_Settings" }; };
        template<typename Archive>
        void serialize(Archive &)  {}
    };
    template<typename prec>
    inline std::string getTypeNameDescription(const NeelProblemSettings<prec>&)
    {
        return NeelProblemSettings<prec>::getSectionName();
    }


    template<typename prec>
    class NeelSphericalProblemSettings
    {
    public:
        bool mUseCoordinateTransformation{ false };
        prec mMinAngleBeforeTransformation{ std::numeric_limits<prec>::epsilon() };

        static std::string getSectionName() { return std::string{ "Neel_Problem_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        { /*Nothing to do in this method yet*/
            ar(Archives::createNamedValue("Use_coordinate_transformation", mUseCoordinateTransformation));
            ar(Archives::createNamedValue("Min_angle_transformation", mMinAngleBeforeTransformation));

            assert(mMinAngleBeforeTransformation >= 0.0);
            assert(mMinAngleBeforeTransformation <= std::acos(-1));
        }
    };

    template<typename prec>
    inline std::string getTypeNameDescription(const NeelSphericalProblemSettings<prec>&)
    {
        return NeelSphericalProblemSettings<prec>::getSectionName();
    } 

    template <typename prec, IProblem value>
    struct problem_settings_mapper
    {
        using type = typename Settings::ProblemSettingsSelector<value>::template type<prec>;
    };
    template<typename prec>
    struct ProblemSettingsWrapper
    {
        template<IProblem value>
        using ProblemSettingsSelector = problem_settings_mapper<prec,value>;
        using ProblemSettings = MyCEL::enum_variant<IProblem, ProblemSettingsSelector, ValidIProblemValues>;

        ProblemSettings settings {{IProblem::Problem_Neel}, {}};
    };
    template <typename prec, typename Archive>
    void serialize(ProblemSettingsWrapper<prec>& wrapper, Archive& ar)
    {
        ar(::SerAr::createNamedEnumVariant(wrapper.settings));
    }
    template <typename prec>
    inline std::string getTypeNameDescription(const ProblemSettingsWrapper<prec> &)
    {
        return "Problem_Settings";
    }
}

#endif	// INC_ProblemSettings_H
// end of ProblemSettings.h
///---------------------------------------------------------------------------------------------------
