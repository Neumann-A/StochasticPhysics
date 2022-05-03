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
        bool                                            m_useRandomInitialParticlePosition{ false };
        bool                                            m_useRandomInitialParticleOrientation{ true };
        bool                                            m_useRandomInitialMagnetisationDir{ true };

        //Starting Orientations
        Vec3D                                            m_initialParticlePosition{ Vec3D::Zero() };
        Vec3D                                            m_initialParticleOrientation{ Vec3D::Zero() };
        Vec3D                                            m_initialMagnetisationDirection{ Vec3D::Zero() };


    public:
        // Access the UseRandomInitialParticlePosition
        inline bool getUseRandomInitialParticlePosition(void) const noexcept { return(m_useRandomInitialParticlePosition); };
        inline void setUseRandomInitialParticlePosition(bool useRandomInitialParticlePosition) noexcept { m_useRandomInitialParticlePosition = useRandomInitialParticlePosition; };
        // Access the InitialParticlePosition
        inline const Vec3D& getInitialParticlePosition(void) const noexcept { return m_initialParticlePosition; };
        inline void setInitialParticlePosition(const Vec3D& initialParticlePosition) noexcept { m_initialParticlePosition = initialParticlePosition; };
        // Access the UseRandomInitialParticleOrientation
        inline bool getUseRandomInitialParticleOrientation(void) const noexcept { return(m_useRandomInitialParticleOrientation); };
        inline void setUseRandomInitialParticleOrientation(bool useRandomInitialParticleOrientation) noexcept { m_useRandomInitialParticleOrientation = useRandomInitialParticleOrientation; };
        // Access the InitialParticleOrientation
        inline const Vec3D& getInitialParticleOrientation(void) const noexcept { return(m_initialParticleOrientation); };
        inline void setInitialParticleOrientation(const Vec3D& initialParticleOrientation) noexcept { m_initialParticleOrientation = initialParticleOrientation; };
        // Access the UseRandomInitialMagnetisationDir
        inline bool getUseRandomInitialMagnetisationDir(void) const noexcept { return(m_useRandomInitialMagnetisationDir); };
        inline void setUseRandomInitialMagnetisationDir(bool useRandomInitialMagnetisationDir) noexcept { m_useRandomInitialMagnetisationDir = useRandomInitialMagnetisationDir; };
        // Access the InitialMagnetisationDirection
        inline const Vec3D& getInitialMagnetisationDirection(void) const noexcept { return(m_initialMagnetisationDirection); };
        inline void setInitialMagnetisationDirection(const Vec3D& initialMagnetisationDirection) noexcept { m_initialMagnetisationDirection = initialMagnetisationDirection; };


        ParticleSimulationInitSettings(bool RandPos, bool RandOrientation, bool RandMagDir, const Vec3D& Pos, const Vec3D& Dir, const Vec3D& MagDir)
            : m_useRandomInitialParticlePosition(RandPos), m_useRandomInitialParticleOrientation(RandOrientation), m_useRandomInitialMagnetisationDir(RandMagDir),
            m_initialParticlePosition(Pos), m_initialParticleOrientation(Dir), m_initialMagnetisationDirection(MagDir)
        {};
        ParticleSimulationInitSettings() = default;

        static std::string getSectionName() { return std::string{ "Particle_Simulation_Initialization" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Use_random_initial_particle_position", m_useRandomInitialParticlePosition));
            ar(Archives::createNamedValue("Use_random_initial_particle_orientation", m_useRandomInitialParticleOrientation));
            ar(Archives::createNamedValue("Use_random_initial_particle_magnetisation", m_useRandomInitialMagnetisationDir));

            if (!m_useRandomInitialParticlePosition)
            {
                ar(Archives::createNamedValue("Initial_particle_position", m_initialParticlePosition));
            }

            if (!m_useRandomInitialParticleOrientation)
            {
                ar(Archives::createNamedValue("Initial_particle_orientation", m_initialParticleOrientation));
            }
            
            if (!m_useRandomInitialMagnetisationDir)
            {
                ar(Archives::createNamedValue("Initial_magnetisation_direction", m_initialMagnetisationDirection));
            }
        }
    };
}
#endif    // INC_ParticleInitSettings_H
// end of ParticleInitSettings.h
///---------------------------------------------------------------------------------------------------
