///-------------------------------------------------------------------------------------------------
// file:	SystemMatrixSettings.h
//
// summary:	Declares the system matrix settings class
///-------------------------------------------------------------------------------------------------


#pragma once
#include <cmath>
#include <exception>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>

#include <Eigen/Core>

#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/LoadConstructor.h>
#include <SerAr/Core/NamedValue.h>

namespace Settings
{
    template<typename prec>
    class PhantomSettings
    {
    private:
        using ThisClass = PhantomSettings<prec>;
        using Vec3D = Eigen::Matrix<prec, 3, 1>;
        using SliceVec = Eigen::Matrix<std::size_t, 3, 1>;
        using VoxelList =Eigen::Matrix<std::size_t,3,Eigen::Dynamic>;

        Vec3D                   mStartfield {Vec3D::Zero()};        // Start-Ecke der Systemmatrix
        Vec3D                   mStopfield {Vec3D::Ones()};         // Stop-Ecke der Systemmatrix
        SliceVec                mSlices{ SliceVec::Zero() };        // Vektor, der sagt, wie viele Voxel in x-, y-, z-Richtung erzeugt werden sollen
        VoxelList               Phantom {{0,0,0}};

            
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

        auto getNumberVoxelsToSimulate(void) const noexcept
        {
                return  Phantom.cols();
        }


        std::vector<std::tuple<SliceVec, Vec3D>> generateFieldsAndVoxels() const
        {
            std::vector<std::tuple<SliceVec, Vec3D>> result;
            //auto Voxels = generateSimulationVoxels();
            //std::cout << "Number of Voxels:" << Voxels.size() << std::endl;
            const auto VoxelSize = getVoxelSize();
            for(int i=0;i<Phantom.cols();i++)
            {
                Vec3D field;
                Vec3D index=(Phantom.col(i)).template cast<typename Vec3D::Scalar>();
                field = mStartfield + VoxelSize.cwiseProduct(index); //.row(i).transpose()
                //field = mStartfield + VoxelSize.cwiseProduct(Phantom.template cast<typename Vec3D::Scalar>());
                result.push_back(std::make_tuple(std::move(Phantom.col(i)), std::move(field))); //.row(i).transpose()
            }
            return result;
        }

        PhantomSettings(const Vec3D& startfield, const Vec3D& stopfield, const SliceVec& slices, const VoxelList list)
            : mStartfield(startfield), mStopfield(stopfield), mSlices(slices),Phantom(list){};

        PhantomSettings(const PhantomSettings& tocopy)
            : mStartfield(tocopy.mStartfield)
            , mStopfield(tocopy.mStopfield)
            , mSlices(tocopy.mSlices)
            , Phantom(tocopy.Phantom){};

        PhantomSettings() = default;

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

        static std::string getSectionName() { return std::string{ "Phantom_Settings" }; };
        
        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Startfield", mStartfield));
            ar(Archives::createNamedValue("Stopfield", mStopfield));
            ar(Archives::createNamedValue("Slices", mSlices));
            // check if slices are correctly set!
            ar(Archives::createNamedValue("Voxels",Phantom));
        }
 };
}
namespace Archives
{
    template <typename prec>
    class LoadConstructor<Settings::PhantomSettings<prec>>
    {
    public:
        using type = Settings::PhantomSettings<prec>;

        template <typename Archive>
        static inline type construct(InputArchive<Archive>& ar)
        {
            type PSettings;

            ar(Archives::createNamedValue(type::getSectionName(), PSettings));

            type ConstructedType{PSettings};
            return ConstructedType;
        }
    };
} // namespace Archives

