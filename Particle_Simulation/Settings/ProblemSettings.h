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

#include <SerAr/Core/NamedValue.h>
#include <SerAr/Core/LoadConstructor.h>
#include <SerAr/Core/OutputArchive.h>
#include <SerAr/Core/InputArchive.h>

namespace Settings
{
    //Forward declrations of problem settings:
    template<typename prec>
    class BrownAndNeelProblemSettings;
    template<typename prec>
    class NeelProblemSettings;
    template<typename prec>
    class NeelSphericalProblemSettings;
    template<typename prec>
    class BrownAndNeelEulerSphericalProblemSettings;


    //TODO: Find a more maintainable and extensible solution for this enum 
    //		which can also be used in templates! (Solver, Problem, Field)
    /// <summary>	Values that represent differen Problems to simulate. </summary>
    enum class IProblem { Problem_undefined, Problem_BrownAndNeel, Problem_BrownAndNeelEulerSpherical, Problem_Neel, Problem_NeelSpherical, Problem_NeelQuaternion	};

// #ifdef _MSC_VER
// #pragma warning (push)
// #pragma warning (disable : 4592) // Disable stupid VS Debug message (could be unecessary since vs2017)
// #endif
    /// <summary>	Map used to change the IProblem enum to a string and vice versa. </summary>
    extern const std::map<IProblem, std::string> IProblemMap;
// #ifdef _MSC_VER
// #pragma warning (pop)
// #endif

    ///-------------------------------------------------------------------------------------------------
    /// <summary>	Gets the enum IProblem from a string. </summary>
    ///
    /// <param name="AnisoString">	The string to transform </param>
    ///
    /// <returns>	An Enum representing the string  </returns>
    ///-------------------------------------------------------------------------------------------------
    std::string to_string(const IProblem&);
    template<typename T>
    T from_string(const std::string&);
    template<>
    IProblem from_string<IProblem>(const std::string&);

    template<typename prec>
    class IProblemSettings
    {
    private:
        typedef IProblemSettings<prec>	ThisClass;

    protected:
        //virtual void WriteValuesToConfigFileDerived(ConfigFile &file, const std::string &section) const = 0;
        //virtual void InsertIntoMATLABarrayDerived(mxArray& mat) const = 0;
        IProblemSettings() = default;
        //virtual ThisClass* getNewProblemSettings(const ConfigFile &file, const std::string &section) = 0;
        IProblemSettings(const ThisClass&) = default;
        ThisClass& operator=(const ThisClass&) = default;

    public:
        virtual IProblem getProblemType() const noexcept = 0;
        virtual ~IProblemSettings() = default;

        virtual std::unique_ptr<ThisClass> clone() const noexcept = 0;

        static inline std::string getSectionName() { return std::string{ "Problem_Settings" }; };
        static inline std::string getProblemString() { return std::string{ "Problem_type" }; };

        
        template< typename Archive>
        std::enable_if_t<std::is_base_of<::Archives::OutputArchive<std::decay_t<Archive>>, std::decay_t<Archive>>::value> serialize(Archive& ar)
        {
            const auto Problem = this->getProblemType();
            const std::string problemstr = ::Settings::to_string(this->getProblemType());
            ar(Archives::createNamedValue(ThisClass::getProblemString(),problemstr));
            
            switch (Problem)
            {
            case Settings::IProblem::Problem_BrownAndNeel:
            {
                dynamic_cast<Settings::BrownAndNeelProblemSettings<prec>&>(*this).serialize(ar);
                break;
            }
            case Settings::IProblem::Problem_Neel:
            {
                dynamic_cast<Settings::NeelProblemSettings<prec>&>(*this).serialize(ar);
                break;
            }
            case Settings::IProblem::Problem_NeelSpherical:
            {
                dynamic_cast<Settings::NeelSphericalProblemSettings<prec>&>(*this).serialize(ar);
                break;
            }
            case Settings::IProblem::Problem_BrownAndNeelEulerSpherical:
            {
                dynamic_cast<Settings::BrownAndNeelEulerSphericalProblemSettings<prec>&>(*this).serialize(ar);
                break;
            }
            default:
            {
                break;
            }
            }

        }
    };

    template<typename prec>
    class BrownAndNeelProblemSettings : public IProblemSettings<prec>
    {
    private:
        bool _UseSimpleModel{ false };
        typedef BrownAndNeelProblemSettings<prec>	ThisClass;
        typedef IProblemSettings<prec>				ProblemInterface;

    protected:

    public:
        BrownAndNeelProblemSettings(const ThisClass&) = default;
        ThisClass& operator=(const ThisClass&) = default;

        IProblem getProblemType() const noexcept override final
        {
            return IProblem::Problem_BrownAndNeel;
        };

        std::unique_ptr<ProblemInterface> clone() const noexcept override final
        {
            return std::make_unique<ThisClass>(*this);
        }

        BrownAndNeelProblemSettings(bool usesimple) : _UseSimpleModel(usesimple) {};
        BrownAndNeelProblemSettings() = default;

        static std::string getSectionName() { return std::string{ "BrownAndNeel_Problem_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Use_simple_model", _UseSimpleModel));
        }

        //static ThisClass createObjectFromConfigFile(const ConfigFile &file, const std::string &section)
        //{
        //	bool useSimple{ file.getBooleanOfKeyInSection(section, SMEMBERNAME(_UseSimpleModel)) };
        //	return ThisClass{ useSimple };
        //};

        // Access the UseSimpleModel
        bool getUseSimpleModel(void) const noexcept { return(_UseSimpleModel); }
        void setUseSimpleModel(bool useSimpleModel)	noexcept { _UseSimpleModel = useSimpleModel; }
    };

    template<typename prec>
    class BrownAndNeelEulerSphericalProblemSettings : public IProblemSettings<prec>
    {
    private:
        typedef BrownAndNeelEulerSphericalProblemSettings<prec>	ThisClass;
        typedef IProblemSettings<prec>				ProblemInterface;

    protected:

    public:
        BrownAndNeelEulerSphericalProblemSettings(const ThisClass&) = default;
        ThisClass& operator=(const ThisClass&) = default;

        bool mUseSphericalCoordinateTransformation{ false };
        bool mUseEulerCoordinateTransformation{ false };
        prec mNeelMinAngleBeforeTransformation{ std::numeric_limits<prec>::epsilon() };
        prec mBrownMinAngleBeforeTransformation{ std::numeric_limits<prec>::epsilon() };

        IProblem getProblemType() const noexcept override final
        {
            return IProblem::Problem_BrownAndNeelEulerSpherical;
        };

        std::unique_ptr<ProblemInterface> clone() const noexcept override final
        {
            return std::make_unique<ThisClass>(*this);
        }

        BrownAndNeelEulerSphericalProblemSettings() = default;

        static std::string getSectionName() { return std::string{ "BrownAndNeelEulerSpherical_Problem_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Use_spherical_coordinate_transformation", mUseSphericalCoordinateTransformation));
            ar(Archives::createNamedValue("Use_euler_coordinate_transformation", mUseEulerCoordinateTransformation));
            ar(Archives::createNamedValue("Neel_Min_angle_before_transformation", mNeelMinAngleBeforeTransformation));
            ar(Archives::createNamedValue("Brown_Min_angle_before_transformation", mBrownMinAngleBeforeTransformation));

            assert(mNeelMinAngleBeforeTransformation >= 0.0);
            assert(mNeelMinAngleBeforeTransformation <= std::acos(-1));
            assert(mBrownMinAngleBeforeTransformation >= 0.0);
            assert(mBrownMinAngleBeforeTransformation <= std::acos(-1));
        }
    };

    template<typename prec>
    class NeelProblemSettings : public IProblemSettings<prec>
    {

    private:
        typedef NeelProblemSettings<prec>			ThisClass;
        typedef IProblemSettings<prec>				ProblemInterface;

    protected:

    public:

        std::unique_ptr<ProblemInterface> clone() const noexcept override final
        {
            return std::make_unique<ThisClass>(*this);
        }

        static std::string getSectionName() { return std::string{ "Neel_Problem_Settings" }; };

        template<typename Archive>
        void serialize(Archive &) 
        { /*Nothing to do in this method yet*/ }

    
        IProblem getProblemType() const noexcept override final
        {
            return IProblem::Problem_Neel;
        };

    };

    template<typename prec>
    class NeelSphericalProblemSettings : public IProblemSettings<prec>
    {

    private:
        typedef NeelSphericalProblemSettings<prec>			ThisClass;
        typedef IProblemSettings<prec>				ProblemInterface;
        
    protected:

    public:

        bool mUseCoordinateTransformation{ false };
        prec mMinAngleBeforeTransformation{ std::numeric_limits<prec>::epsilon() };

        std::unique_ptr<ProblemInterface> clone() const noexcept override final
        {
            return std::make_unique<ThisClass>(*this);
        }

        static std::string getSectionName() { return std::string{ "Neel_Problem_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        { /*Nothing to do in this method yet*/
            ar(Archives::createNamedValue("Use_coordinate_transformation", mUseCoordinateTransformation));
            ar(Archives::createNamedValue("Min_angle_transformation", mMinAngleBeforeTransformation));

            assert(mMinAngleBeforeTransformation >= 0.0);
            assert(mMinAngleBeforeTransformation <= std::acos(-1));
        }


        IProblem getProblemType() const noexcept override final
        {
            return IProblem::Problem_NeelSpherical;
        };

    };

}

namespace Archives
{
    template<typename ToConstruct>
    class LoadConstructor;

    template<typename prec>
    class LoadConstructor<typename Settings::template IProblemSettings<prec>>
    {
    private:
        using ToConstruct = typename Settings::template IProblemSettings<prec>;
    public:
        template <typename Archive>
        static inline decltype(auto) construct(InputArchive<Archive>& ar)
        {
            auto& arch = static_cast<typename InputArchive<Archive>::ArchiveType&>(ar);
            
            std::string ProblemType;
            arch(Archives::createNamedValue(ToConstruct::getSectionName(),Archives::createNamedValue(ToConstruct::getProblemString(), ProblemType)));
            const auto type{ Settings::from_string<Settings::IProblem>(ProblemType) };
            std::unique_ptr<ToConstruct> tmp{ nullptr };
            
            switch (type)
            {
            case Settings::IProblem::Problem_Neel:
            {
                Settings::NeelProblemSettings<prec> set;
                arch(Archives::createNamedValue(ToConstruct::getSectionName(), set));
                tmp = std::make_unique<Settings::NeelProblemSettings<prec>>(set);
                break;
            }
            case Settings::IProblem::Problem_NeelSpherical:
            {
                Settings::NeelSphericalProblemSettings<prec> set;
                arch(Archives::createNamedValue(ToConstruct::getSectionName(), set));
                tmp = std::make_unique<Settings::NeelSphericalProblemSettings<prec>>(set);
                break;
            }
            case Settings::IProblem::Problem_BrownAndNeel:
            {
                Settings::BrownAndNeelProblemSettings<prec> set;
                arch(Archives::createNamedValue(ToConstruct::getSectionName(), set));
                tmp = std::make_unique<Settings::BrownAndNeelProblemSettings<prec>>(set);
                break;
            }
            case Settings::IProblem::Problem_BrownAndNeelEulerSpherical:
            {
                Settings::BrownAndNeelEulerSphericalProblemSettings<prec> set;
                arch(Archives::createNamedValue(ToConstruct::getSectionName(), set));
                tmp = std::make_unique<Settings::BrownAndNeelEulerSphericalProblemSettings<prec>>(set);
                break;
            }
            default:
                tmp = nullptr;
                break;
            }
            return tmp;
        }
    
    };
}


#endif	// INC_ProblemSettings_H
// end of ProblemSettings.h
///---------------------------------------------------------------------------------------------------
