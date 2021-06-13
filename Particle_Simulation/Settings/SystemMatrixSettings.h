///-------------------------------------------------------------------------------------------------
// file:	SystemMatrixSettings.h
//
// summary:	Declares the system matrix settings class
///-------------------------------------------------------------------------------------------------

#ifndef INC_SystemMatrixSettings_H
#define INC_SystemMatrixSettings_H

#pragma once
#include <cmath>
#include <exception>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>


#include <Eigen/Core>

#include <SerAr/Core/NamedValue.h>

namespace Settings
{
    template<typename prec>
    bool isApproxEqual(prec a, prec b)
    {
        if( (std::abs)(a-b) <= std::numeric_limits<prec>::epsilon() )
            return true;
        return (std::abs)(a-b) <= std::numeric_limits<prec>::epsilon() * (std::max)(a,b);
    }

    template<typename prec>
    class SystemMatrixSettings
    {
    private:
        using ThisClass = SystemMatrixSettings<prec>;
        using Vec3D = Eigen::Matrix<prec, 3, 1>;
        using SliceVec = Eigen::Matrix<std::size_t, 3, 1>;


        Vec3D					mStartfield {Vec3D::Zero()};					// Start-Ecke der Systemmatrix
        Vec3D					mStopfield {Vec3D::Zero()};						// Stop-Ecke der Systemmatrix
        SliceVec				mSlices{ SliceVec::Zero() };						// Vektor, der sagt, wie viele Voxel in x-, y-, z-Richtung erzeugt werden sollen
        bool					mUseStartStopVoxels{ false };	// Should Voxel be jumped?
        SliceVec				mStartVoxel{ SliceVec::Zero() };	// Where to Start the System Matrix
        SliceVec				mStopVoxel{ SliceVec::Zero() };	// where to Stop the System Matrix

        inline void checkSlices()
        {
            if (mSlices(0) < 1 || mSlices(1) < 1 || mSlices(2) < 1)
            {
                //Logger::Error("Die Anzahl der Voxel muss in allen Dimensionen mind. 1 betragen - auch bei 2D-Systemfunktionen!");
                std::runtime_error{ "SystemMatrixSettings: Number of Voxels must be at least one for every dimension!" };
            }

            //Check if Start and StopField makes sense
            auto sliceno = (mSlices.array() - 1).matrix().eval();
            if (sliceno(0) == 0)
            {
                if ( isApproxEqual(mStopfield(0),mStartfield(0)))
                {
                    throw std::runtime_error{ "SystemMatrixSettings: Invalid Parameters. Unable to calculate VoxelSize. x-Direction." };
                }
                sliceno(0) = 1;
            }
            if (sliceno(1) == 0)
            {
                if ( isApproxEqual(mStopfield(1),mStartfield(1)) )
                {
                    throw std::runtime_error{ "SystemMatrixSettings: Invalid Parameters. Unable to calculate VoxelSize. y-Direction." };
                }
                sliceno(1) = 1;
            }
            if (sliceno(2) == 0)
            {
                if ( isApproxEqual(mStopfield(2),mStartfield(2)) )
                {
                    throw std::runtime_error{ "SystemMatrixSettings: Invalid Parameters. Unable to calculate VoxelSize. z-Direction." };
                }
                sliceno(2) = 1;
            }
        }

        inline void checkStartVoxel()
        {
            if (mSlices(0) < mStartVoxel(0) || mSlices(1) < mStartVoxel(1) || mSlices(2) < mStartVoxel(2))
            {
                std::runtime_error{ "SystemMatrixSettings: Startvoxel bigger than defined slices " };
            }
        }

        inline void checkStopVoxel()
        {

            if (mSlices(0) < mStopVoxel(0) || mSlices(1) < mStopVoxel(1) || mSlices(2) < mStopVoxel(2))
            {
                std::runtime_error{ "SystemMatrixSettings: Stopvoxel bigger than defined slices " };
            }

            if (mStartVoxel(2) < mStopVoxel(2))
            {
                return; //Everything ok
            }
            else if (mStartVoxel(2) == mStopVoxel(2))
            {
                if (mStartVoxel(1) < mStopVoxel(1))
                {
                    return; //Everything ok
                }
                else if (mStartVoxel(1) == mStopVoxel(1))
                {
                    if (mStartVoxel(0) < mStopVoxel(0))
                    {
                        return; //Everything ok
                    }
                    if (mStartVoxel(0) == mStopVoxel(0))
                    {
                        std::runtime_error{ "SystemMatrixSettings: Simulation with SystemMatrix Settings does not make sense! Consider doing one without! (StartVoxel = StopVoxel) " };
                    }
                    else
                    {
                        std::runtime_error{ "SystemMatrixSettings: Invalid Stopvoxel (Logical not behind Startvoxel; x-Direction)  " };
                    }
                }
                else
                {
                    std::runtime_error{ "SystemMatrixSettings: Invalid Stopvoxel (Logical not behind Startvoxel; y-Direction)  " };
                }
            }
            else
            {
                std::runtime_error{ "SystemMatrixSettings: Invalid Stopvoxel (Logical not behind Startvoxel; z-Direction)  " };
            }

        }


    public:
        // Access Startfield
        inline const Vec3D& getStartfield(void) const noexcept { return mStartfield; };
        inline void setStartfield(const Vec3D& uebergabe_mStartfield) noexcept { mStartfield = uebergabe_mStartfield; };
        // Access Stopfield
        inline const Vec3D& getStopfield(void) const noexcept { return mStopfield; };
        inline void setStopfield(const Vec3D& uebergabe_mStopfield) noexcept { mStopfield = uebergabe_mStopfield; };
        // Access mSlices
        inline const SliceVec& getSlices(void) const noexcept { return mSlices; };
        inline void setSlices(const SliceVec& uebergabe_mSlices) noexcept { mSlices = uebergabe_mSlices; };
        
        // Access the StartVoxel
        inline const SliceVec& getStartVoxel(void) const noexcept { return(mStartVoxel); }
        inline void setStartVoxel(const SliceVec& startVoxel) noexcept { mStartVoxel = startVoxel; }

        // Access the StopVoxel
        inline const SliceVec& getStopVoxel(void) const noexcept { return(mStopVoxel); }
        inline void setStopVoxel(const SliceVec& stopVoxel) noexcept { mStopVoxel = stopVoxel; }

        // Access the UseStartStopVoxels
        inline bool getUseStartStopVoxels(void) const noexcept { return(mUseStartStopVoxels); }
        inline void setUseStartStopVoxels(bool useStartStopVoxels)	noexcept { mUseStartStopVoxels = useStartStopVoxels; }

        auto getNumberVoxelsToSimulate(void) const noexcept
        {
            if (mUseStartStopVoxels)
            {
                //const auto& xVoxels = mSlices(0);
                //const auto& yVoxels = mSlices(1);
                //const auto& zVoxels = mSlices(2);

                auto zplanevoxel = mStartVoxel(2) > mStopVoxel(2) ? mStartVoxel(2) - mStopVoxel(2) : mStopVoxel(2) - mStartVoxel(2); //To avoid integer wrap around!
                zplanevoxel *= mSlices(0)*mSlices(1);

                auto yplanevoxel = mStartVoxel(1) > mStopVoxel(1) ? mStartVoxel(1) - mStopVoxel(1) : mStopVoxel(1) - mStartVoxel(1); //To avoid integer wrap around!
                yplanevoxel *= mSlices(0);

                const auto xplanevoxel = mStartVoxel(0) > mStopVoxel(0) ? mStartVoxel(0) - mStopVoxel(0) : mStopVoxel(0) - mStartVoxel(0); //To avoid integer wrap around!

                const auto result = mStartVoxel(0) > mStopVoxel(0) ? yplanevoxel + zplanevoxel - xplanevoxel : xplanevoxel + yplanevoxel + zplanevoxel; // Be carefull with the minus!
                 
                return result;
            }
            else
            {
                return  mSlices(0)*mSlices(1)*mSlices(2);
            }
        }

        std::vector<SliceVec> generateSimulationVoxels() const
        {
            std::vector<SliceVec> result;

            std::size_t counterVoxels{ 0 };
            SliceVec startVoxel  {mStartVoxel};

            const auto noVoxelSim{ getNumberVoxelsToSimulate() };
            //std::cout << "Number of Voxels to create:" << noVoxelSim << std::endl;

            for (std::size_t zInd = startVoxel(2); zInd < mSlices(2) && counterVoxels < noVoxelSim; ++zInd)
            {
                for (std::size_t yInd = startVoxel(1); yInd < mSlices(1) && counterVoxels < noVoxelSim; ++yInd)
                {
                    for (std::size_t xInd = startVoxel(0); xInd < mSlices(0) && counterVoxels < noVoxelSim; ++xInd)
                    {
                        counterVoxels++;
                        SliceVec Voxel;
                        Voxel(0) = xInd;
                        Voxel(1) = yInd;
                        Voxel(2) = zInd;
                        result.push_back(Voxel);
                    }
                    startVoxel(0) = 0;
                }
                startVoxel(1) = 0;
            }
            //std::cout << "Number of Voxels created:" << result.size() << std::endl;
            return result;
        }

        std::vector<std::tuple<SliceVec, Vec3D>> generateFieldsAndVoxels() const
        {
            std::vector<std::tuple<SliceVec, Vec3D>> result;
            auto Voxels = generateSimulationVoxels();
            //std::cout << "Number of Voxels:" << Voxels.size() << std::endl;
            const auto VoxelSize = getVoxelSize();
            for(auto& Voxel : Voxels)
            {
                Vec3D field;
                field = mStartfield + VoxelSize.cwiseProduct(Voxel.template cast<typename Vec3D::Scalar>());
                result.push_back(std::make_tuple(std::move(Voxel), std::move(field)));
            }
            return result;
        }

        SystemMatrixSettings(const Vec3D& startfield, const Vec3D& stopfield, const SliceVec& slices, bool UseStartVoxel = false, const SliceVec& StopVoxel = SliceVec::Ones(), const SliceVec& StartVoxel = SliceVec::Zero())
            : mStartfield(startfield), mStopfield(stopfield), mSlices(slices), mUseStartStopVoxels(std::move(UseStartVoxel)), mStartVoxel(StartVoxel), mStopVoxel(StopVoxel)
        {
            checkSlices();
            if (mUseStartStopVoxels)
            {
                checkStartVoxel();
                checkStopVoxel();
            }
            else
            {
                mStopVoxel = (mSlices - SliceVec::Ones());
            }
        };
        SystemMatrixSettings() = default;

        inline Vec3D getVoxelSize() const noexcept
        {
            const Vec3D Size_FOV{ (mStopfield - mStartfield) };

            auto sliceno = (mSlices.array() - 1).matrix().eval();
                
            if (sliceno(0) == 0)
            {
                sliceno(0) = 1;
            }
            if (sliceno(1) == 0)
            {
                sliceno(1) = 1;
            }
            if (sliceno(2) == 0)
            {
                sliceno(2) = 1;
            }

            //Voxelsize is Size/(NumberOfSlices-1)
            Vec3D FOV_VoxelSize{ Size_FOV.cwiseQuotient(sliceno.template cast<typename Vec3D::Scalar>()) };

            return FOV_VoxelSize;
        }

        static std::string getSectionName() { return std::string{ "System_Matrix_Settings" }; };
        
        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Startfield", mStartfield));
            ar(Archives::createNamedValue("Stopfield", mStopfield));

            ar(Archives::createNamedValue("Slices", mSlices));
            // check if slices are correctly set!
            checkSlices();

            ar(Archives::createNamedValue("Use_start_stop_Voxel", mUseStartStopVoxels));

            if (mUseStartStopVoxels)
            {
                ar(Archives::createNamedValue("Start_Voxel", mStartVoxel));
                ar(Archives::createNamedValue("Stop_Voxel", mStopVoxel));
                //Check Start and Stop Voxel
                checkStartVoxel();
                checkStopVoxel();
            }
            else
            {
                mStartVoxel = SliceVec::Zero();
                mStopVoxel = (mSlices - SliceVec::Ones());
                checkStartVoxel();
                checkStopVoxel();
            }


        }
    };
}
#endif	// INC_SystemMatrixSettings_H

