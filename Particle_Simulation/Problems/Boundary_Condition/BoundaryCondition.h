///-------------------------------------------------------------------------------------------------
// file:	BoundaryCondition.h
//
// summary:	Declares the boundary condition class
///-------------------------------------------------------------------------------------------------

#pragma once

#include "Eigen\Core"

///-------------------------------------------------------------------------------------------------
/// <summary>	CRTP class to define static interface for the Boundary class. </summary>
///
/// <remarks>	Neumann, 07.07.2016. </remarks>
///
/// <typeparam name="prec">			  	Type of the prec. </typeparam>
/// <typeparam name="DerivedBoundary">	Type of the derived boundary. </typeparam>
///-------------------------------------------------------------------------------------------------

template <typename Boundary>
class BoundaryTraits;


template <typename DerivedBoundary>
class BaseBoundary
{
protected:
    typedef typename BoundaryTraits<DerivedBoundary>::CheckVector		CheckVector;

    __forceinline DerivedBoundary& thisBoundary() noexcept // Handy helper to remove the static_cast in the other functions
    {
        return *static_cast<DerivedBoundary*>(this);
    };
public:
    
    __forceinline void checkBoundary(CheckVector &xi)
    {
        std::cout << "Warning wrong function call to BaseBoundary" << std::endl;
        return thisBoundary().checkBoundary();
    }


};


#include "BrownianMotionParameters.h"

template <typename prec>
class EllipsoidBoundary : public BaseBoundary<EllipsoidBoundary<prec>>
{
public:
    typedef prec					Precision;
    
private:
    typedef EllipsoidBoundary		ThisClass;
    typedef typename BoundaryTraits<ThisClass>::CheckVector CheckVector;
    typedef typename Eigen::Matrix<Precision, 3, 1>	Vec3D;

    Precision m_a { 1 };
    Precision m_b { 1 };
    Precision m_c { 1 };
    //Vec3D _normal;
    mutable CheckVector _LastVector {CheckVector::Zero()};

public:
    EllipsoidBoundary(const BoundaryTraits<ThisClass>::Parameters& params)
    {
        m_a = params.GetA();
        m_b = params.GetB();
        m_c = params.GetC();

    }


    __forceinline void checkBoundary(CheckVector &xi) const noexcept
    {
        auto tmp = std::pow(xi(0) / m_a, 2) + std::pow(xi(1) / m_b, 2) + std::pow(xi(2) / m_c, 2);

        //TODO: Proper Reflection at Boundary
        if (tmp > 1) // New Vector Outside boundary
        {
            Vec3D dx = xi - _LastVector; // Direction to move
            dx.normalize();
            Vec3D xdx = xi.cwiseProduct(_LastVector); // x * dx
            Vec3D x2 = _LastVector.cwiseProduct(_LastVector);
            Vec3D dx2 = dx.cwiseProduct(dx);
            
            auto a2 = std::pow(m_a, 2);
            auto b2 = std::pow(_b, 2);
            auto c2 = std::pow(_c, 2);

            Vec3D tmp;
            tmp(0) = 2 * b2*c2;
            tmp(1) = 2 * a2*c2;
            tmp(2) = 2 * a2*b2;

            prec pre = tmp.dot(xdx);
            prec pre2 = -tmp.dot(dx2);

            //Calculate the ray to the boundary (used mathematice to get this)
            auto m = (pre - std::sqrt(pow(-pre, 2) - (a2*b2*c2 - tmp.dot(x2))*(pre2))) / pre2;

            Vec3D dxbound{ m*dx }; //Vector from _LastVector to boundary using xi as direction

            Vec3D contactpoint{ dxbound + _LastVector }; // Contactpoint with Boundary
            Vec3D normalatcontact;  
            normalatcontact << 2 / std::pow(m_a, 2)*contactpoint(0), 2 / std::pow(_b, 2)*contactpoint(1), 2 / std::pow(_c, 2)*contactpoint(2); //Normal at Boundary contact point
            normalatcontact.normalize();

            Vec3D reflectvector{ dx - 2 * dx.dot(normalatcontact)*normalatcontact }; //Find the reflection vector
            reflectvector.normalize();
            Vec3D remaining{ xi - _LastVector - dxbound };
            reflectvector *= remaining.norm();

            xi = contactpoint + reflectvector; //Returning the result

            checkBoundary(xi);			// Check the result again
        }
        return;
    }
};

template <typename prec>
class BoundaryTraits<EllipsoidBoundary<prec>>
{
public:
    typedef prec															Precision;
    typedef	typename Problems::BrownianMotionParameters<prec>				Parameters;
    typedef typename Eigen::Matrix<Precision, 3, 1>							CheckVector;
};