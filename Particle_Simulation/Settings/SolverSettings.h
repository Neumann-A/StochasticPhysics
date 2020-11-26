///---------------------------------------------------------------------------------------------------
// file:        SolverSettings.h
//
// summary:     Declares the solver settings class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 25.06.2016

#ifndef INC_SolverSettings_H
#define INC_SolverSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <SerAr/Core/NamedValue.h>

#include <map>
#include <string>


#ifdef WITH_GSL_SOLVERS
#include <MyCEL/math/GSL_Implicit_Solver.h>
#include <MyCEL/math/GSL_Implicit_Solver_Derivative_Free.h>
#else
#include <exception>
#endif

namespace Settings
{

    //TODO: Find a more maintainable and extensible solution for this enum
    //        which can also be used in templates! (Solver, Problem, Field)
    enum class ISolver
    {
        Solver_undefined,
        Solver_EulerMaruyama,
        Solver_EulerMaruyamaNormalized,
        Solver_Implicit_Midpoint,
        Solver_Implicit_Midpoint_GSL,
        Solver_Implicit_Midpoint_GSL_Derivative_Free,
        Solver_Millstein,
        Solver_Heun_Strong,
        Solver_Heun_NotConsistent,
        Solver_WeakTest,
        Solver_ExplicitStrong1_0
    };

    extern const std::map<ISolver, std::string> ISolverMap;
#ifdef WITH_GSL_SOLVERS
    extern const std::map<gsl_solver_type, std::string> IGSLSolverMap;
    extern const std::map<gsl_solver_type_derivative_free, std::string> IGSL2SolverMap;
#endif
    std::string to_string(const ISolver& field);
    template <typename T>
    T from_string(const std::string&);
    template <>
    ISolver from_string<ISolver>(const std::string& string);

#ifdef WITH_GSL_SOLVERS
    std::string to_string(const gsl_solver_type& field);
    template <>
    gsl_solver_type from_string<gsl_solver_type>(const std::string& string);

    std::string to_string(const gsl_solver_type_derivative_free& field);
    template <>
    gsl_solver_type_derivative_free from_string<gsl_solver_type_derivative_free>(const std::string& string);
#endif

    template <typename prec>
    class SolverSettings
    {
        typedef SolverSettings<prec> ThisClass;

    private:
        ISolver TypeOfSolver{ISolver::Solver_undefined};
#ifdef WITH_GSL_SOLVERS
        gsl_solver_type TypeOfImplicitGSLSolver{gsl_solver_type::undefined};
        gsl_solver_type_derivative_free TypeOfImplicitGSL2Solver{gsl_solver_type_derivative_free::undefined};
#endif
        int32_t DoubleNoiseApprox{-1};
        std::size_t MaxIteration{0};
        prec AccuracyGoal{0};

    protected:
    public:
        typedef prec Precision;

        explicit SolverSettings(ISolver solver, int32_t doublenoiseapprox)
            : TypeOfSolver(std::move(solver))
            , DoubleNoiseApprox(std::move(doublenoiseapprox)){};
        SolverSettings(){};

        inline const auto& getTypeOfSolver() const noexcept { return TypeOfSolver; };
        inline const auto& getDoubleNoiseApprox() const noexcept { return DoubleNoiseApprox; };
        inline const auto& getMaxIteration() const noexcept { return MaxIteration; };
        inline const auto& getAccuracyGoal() const noexcept { return AccuracyGoal; };
#ifdef WITH_GSL_SOLVERS
        inline const auto& getImplicitGSLSolverType() const noexcept { return TypeOfImplicitGSLSolver; };
        inline const auto& getImplicitGSL2SolverType() const noexcept { return TypeOfImplicitGSL2Solver; };
#endif

        static inline std::string getSectionName() { return std::string{"Solver_Settings"}; };

        template <typename Archive>
        void serialize(Archive& ar)
        {
            {
                std::string str{to_string(TypeOfSolver)};
                ar(Archives::createNamedValue(std::string{"Solver"}, str));
                TypeOfSolver = from_string<decltype(TypeOfSolver)>(str);
            }
            if (TypeOfSolver == ISolver::Solver_ExplicitStrong1_0 || TypeOfSolver == ISolver::Solver_WeakTest) {
                ar(Archives::createNamedValue(std::string{"Approximation_of_double_noise_integral"},
                                              DoubleNoiseApprox));
            }

            if (TypeOfSolver == ISolver::Solver_Implicit_Midpoint ||
                TypeOfSolver == ISolver::Solver_Implicit_Midpoint_GSL ||
                TypeOfSolver == ISolver::Solver_Implicit_Midpoint_GSL_Derivative_Free) {
                ar(Archives::createNamedValue(std::string{"Max_Iterations"}, MaxIteration));
                ar(Archives::createNamedValue(std::string{"Accuracy_Goal"}, AccuracyGoal));
            }

#ifdef WITH_GSL_SOLVERS
            if (TypeOfSolver == ISolver::Solver_Implicit_Midpoint_GSL) {
                std::string str{to_string(TypeOfImplicitGSLSolver)};
                ar(Archives::createNamedValue(std::string{"Implicit_GSL_Solver"}, str));
                TypeOfImplicitGSLSolver = from_string<decltype(TypeOfImplicitGSLSolver)>(str);
            }
            if (TypeOfSolver == ISolver::Solver_Implicit_Midpoint_GSL_Derivative_Free) {
                std::string str{to_string(TypeOfImplicitGSL2Solver)};
                ar(Archives::createNamedValue(std::string{"Implicit_GSL_Solver_Derivative_Free"}, str));
                TypeOfImplicitGSL2Solver = from_string<decltype(TypeOfImplicitGSL2Solver)>(str);
            }
#else
            if (TypeOfSolver == ISolver::Solver_Implicit_Midpoint_GSL ||
                TypeOfSolver == ISolver::Solver_Implicit_Midpoint_GSL_Derivative_Free) {
                throw std::runtime_error{"Programm was build without GSL solvers! (So dont try to use them!)"};
            }
#endif
        }
    };
} // namespace Settings

#endif // INC_SolverSettings_H
// end of SolverSettings.h
///---------------------------------------------------------------------------------------------------
