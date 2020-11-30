#pragma once

#include <type_traits>

#include <MyCEL/basics/BasicMacros.h>
#include "../Definitions/GeneralProblem_Definitions.h"

namespace Problems::Anisotropy
{
    template<typename T>
    struct AnisotropyTraits
    {
        static_assert(std::is_same_v<void, T>, "Need to define AnisotropyTraits specialization for type T!");
            //Default Traits:
            //using Precisions = prec;
            //using Anisotropy = T;
            //using InputVector = void;
            //static constexpr CoordinateSystem coordsystem = CoordinateSystem::cartesian;
            //static constexpr bool is_specialized_v = false;
    };

    namespace
    {
        //All avaiable traits!
        template<typename T, typename void_t = void>
        constexpr bool defined_coordsystem_v = false;
        template<typename T>
        constexpr bool defined_coordsystem_v<T,std::void_t<decltype(T::coordsystem)>> = true;

        template<typename T, typename void_t = void>
        constexpr bool defined_anisotropy_v = false;
        template<typename T>
        constexpr bool defined_anisotropy_v<T, std::void_t<typename T::Anisotropy>> = true;

        template<typename T, typename void_t = void>
        constexpr bool defined_inputvector_v = false;
        template<typename T>
        constexpr bool defined_inputvector_v<T, std::void_t<typename T::InputVector>> = true;

        template<typename T, typename void_t = void>
        constexpr bool defined_precision_v = false;
        template<typename T>
        constexpr bool defined_precision_v<T, std::void_t<typename T::Precision>> = true;

        template<typename T, typename void_t = void>
        constexpr bool defined_number_of_anisotropy_constants_v = false;
        template<typename T>
        constexpr bool defined_number_of_anisotropy_constants_v<T, std::void_t<decltype(T::number_anisotropy_constants)>> = true;
    }

    template<typename T>
    class GeneralAnisotropy
    {
    private:
        using Derived = T;
        BASIC_ALWAYS_INLINE Derived& anisotropy() BASIC_NOEXCEPT
        {
            return *static_cast<Derived * const>(this);
        };
    public:
        using type = T;
        using traits = AnisotropyTraits<T>;

        //Checks if trait class has been correctly defined!
        static_assert(defined_coordsystem_v<traits>, "Forgot to define coordinate system in traits specialization!");
        static_assert(defined_anisotropy_v<traits>, "Forgot to define Anisotropy in traits specialization!");
        static_assert(defined_inputvector_v<traits>, "Forgot to define InputVector in traits specialization!");
        static_assert(defined_precision_v<traits>, "Forgot to define Precision in traits specialization!");
        static_assert(std::is_floating_point_v<typename traits::Precision>, "Precision must be defined as a valid floating point!");
        static_assert(std::is_same_v<T, std::decay_t<T>>, "Type T should be without additional qualifiers in CRTP pattern");
        static_assert(defined_number_of_anisotropy_constants_v<traits>, "Forgot to define number of anistropy constants in traits specialization!");
    };


    template<typename T>
    constexpr bool is_cartesian_v = (T::traits::coordsystem == CoordinateSystem::cartesian);
    template<typename T>
    constexpr bool is_spherical_v = (T::traits::coordsystem == CoordinateSystem::spherical);
    template<typename T>
    constexpr bool is_polar_v = (T::traits::coordsystem == CoordinateSystem::polar);

    template<typename T, typename void_t>
    constexpr bool is_specialized_v = true;
    template<typename T>
    constexpr bool is_specialized_v<T, std::void_t<decltype(T::traits::is_specialized)>> = T::traits::is_specialized_v;
}
