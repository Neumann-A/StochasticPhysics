#pragma once

#include <type_traits>

#include "basics/BasicMacros.h"
#include "../Definitions/General_Definitions.h"

namespace Problems::Anisotropy
{
	template<typename T>
	struct AnisotropyTraits
	{
		static_assert(std::is_same_v<T, std::decay_t<T>>, "Type T should be without additional qualifiers! CRTP Pattern");
		static_assert(std::is_same_v<typename T::traits, AnisotropyTraits<T>>, "T must define this class as it traits type!");

		//Default Traits:
		using Anisotropy = T;
		using InputVector = typename T::InputVector;
		static constexpr CoordinateSystem coordsystem = CoordinateSystem::cartesian;
		static constexpr is_specialized_v = false;
	};

	namespace
	{
		template<typename T, typename void_t>
		constexpr bool defined_coordsystem_v = false;
		template<typename T>
		constexpr bool defined_coordsystem_v<T,std::void_t<decltype(T::coordsystem)>> = true;

		template<typename T, typename void_t>
		constexpr bool defined_anisotropy_v = false;
		template<typename T>
		constexpr bool defined_anisotropy_v<T, std::void_t<typename T::Anisotropy>> = true;

		template<typename T, typename void_t>
		constexpr bool defined_inputvector_v = false;
		template<typename T>
		constexpr bool defined_inputvector_v<T, std::void_t<typename T::InputVector>> = true;
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

		using InputVector = typename traits::InputVector;

	

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