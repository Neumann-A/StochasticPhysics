#pragma once

#ifndef INC_ISINGLEPARTICLESIMULATOR_H_
#define INC_ISINGLEPARTICLESIMULATOR_H_

#include <MyCEL/basics/BasicMacros.h>

#include "Simulator.hpp"

class ISingleParticleSimulator
{
    MY_INTERFACE(ISingleParticleSimulator)
private:
protected:
public:
    virtual bool doSimulation(const uint64_t &NumberOfSteps, const uint64_t &OverSampling) = 0;
    virtual void resetSimulation() = 0;
};

#endif //_ISINGLEPARTICLESIMULATOR_H_
