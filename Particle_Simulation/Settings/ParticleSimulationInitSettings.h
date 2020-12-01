///---------------------------------------------------------------------------------------------------
// file:        ParticleSimulationInitSettings.h
//
// summary:     Declares the initial particle settings for the simulation
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 07.06.2016

#ifndef INC_ParticleSimulationInitSettings_H
#define INC_ParticleSimulationInitSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once
#include <SerAr/Core/NamedValue.h>
#include "../General/MathTypes.hpp"



namespace Settings
{
    template<typename prec>
    class ParticleSimulationInitSettings
    {
    private:
        using ThisClass = ParticleSimulationInitSettings<prec>;
        using Vec3D = SPhys::math::Vector3D<prec>;


        //Use Starting Orientations?
        bool                                            _useRandomInitialParticlePosition{ false };
        bool                                            _useRandomInitialParticleOrientation{ true };
        bool                                            _useRandomInitialMagnetisationDir{ true };

        //Starting Orientations
        Vec3D                                            _initialParticlePosition{ Vec3D::Zero() };
        Vec3D                                            _initialParticleOrientation{ Vec3D::Zero() };
        Vec3D                                            _initialMagnetisationDirection{ Vec3D::Zero() };


    public:
        // Access the UseRandomInitialParticlePosition
        inline bool getUseRandomInitialParticlePosition(void) const noexcept { return(_useRandomInitialParticlePosition); };
        inline void setUseRandomInitialParticlePosition(bool useRandomInitialParticlePosition) noexcept { _useRandomInitialParticlePosition = useRandomInitialParticlePosition; };
        // Access the InitialParticlePosition
        inline const Vec3D& getInitialParticlePosition(void) const noexcept { return _initialParticlePosition; };
        inline void setInitialParticlePosition(const Vec3D& initialParticlePosition) noexcept { _initialParticlePosition = initialParticlePosition; };
        // Access the UseRandomInitialParticleOrientation
        inline bool getUseRandomInitialParticleOrientation(void) const noexcept { return(_useRandomInitialParticleOrientation); };
        inline void setUseRandomInitialParticleOrientation(bool useRandomInitialParticleOrientation) noexcept { _useRandomInitialParticleOrientation = useRandomInitialParticleOrientation; };
        // Access the InitialParticleOrientation
        inline const Vec3D& getInitialParticleOrientation(void) const noexcept { return(_initialParticleOrientation); };
        inline void setInitialParticleOrientation(const Vec3D& initialParticleOrientation) noexcept { _initialParticleOrientation = initialParticleOrientation; };
        // Access the UseRandomInitialMagnetisationDir
        inline bool getUseRandomInitialMagnetisationDir(void) const noexcept { return(_useRandomInitialMagnetisationDir); };
        inline void setUseRandomInitialMagnetisationDir(bool useRandomInitialMagnetisationDir) noexcept { _useRandomInitialMagnetisationDir = useRandomInitialMagnetisationDir; };
        // Access the InitialMagnetisationDirection
        inline const Vec3D& getInitialMagnetisationDirection(void) const noexcept { return(_initialMagnetisationDirection); };
        inline void setInitialMagnetisationDirection(const Vec3D& initialMagnetisationDirection) noexcept { _initialMagnetisationDirection = initialMagnetisationDirection; };


        ParticleSimulationInitSettings(bool RandPos, bool RandOrientation, bool RandMagDir, const Vec3D& Pos, const Vec3D& Dir, const Vec3D& MagDir)
            : _useRandomInitialParticlePosition(RandPos), _useRandomInitialParticleOrientation(RandOrientation), _useRandomInitialMagnetisationDir(RandMagDir),
            _initialParticlePosition(Pos), _initialParticleOrientation(Dir), _initialMagnetisationDirection(MagDir)
        {};
        ParticleSimulationInitSettings() = default;

        static std::string getSectionName() { return std::string{ "Particle_Simulation_Initialization" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Use_random_initial_particle_position", _useRandomInitialParticlePosition));
            ar(Archives::createNamedValue("Use_random_initial_particle_orientation", _useRandomInitialParticleOrientation));
            ar(Archives::createNamedValue("Use_random_initial_particle_magnetisation", _useRandomInitialMagnetisationDir));

            if (!_useRandomInitialParticlePosition)
            {
                ar(Archives::createNamedValue("Initial_particle_position", _initialParticlePosition));
            }

            if (!_useRandomInitialParticleOrientation)
            {
                ar(Archives::createNamedValue("Initial_particle_orientation", _initialParticleOrientation));
            }
            
            if (!_useRandomInitialMagnetisationDir)
            {
                ar(Archives::createNamedValue("Initial_magnetisation_direction", _initialMagnetisationDirection));
            }
        }
    };
};
#endif    // INC_ParticleInitSettings_H
// end of ParticleInitSettings.h
///---------------------------------------------------------------------------------------------------
