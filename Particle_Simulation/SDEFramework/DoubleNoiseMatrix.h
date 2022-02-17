/* DoubleNoiseMatrix
* Calculates the Noise Matrix which is necessary for explicit Solvers with strong order 1 and higher
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#pragma once

#ifndef INC_DoubleNoiseMatrix_H_
#define INC_DoubleNoiseMatrix_H_

#include <cmath>
#include <random>
#include <array>
#include <limits>

#include <Eigen/Core>

#ifdef USE_BOOST_RANDOM
#include <boost/random/normal_distribution.hpp>
//#include <boost/random.hpp>
#endif

#ifdef USE_PCG_RANDOM
#include <pcg_extras.hpp>
#include <pcg_random.hpp>
#endif

#define M_1_DIV_2_PISQR 0.05066059182116888572193973160486 // 1/(2 Pi^2)
#define M_1_DIV_2PI 0.15915494309189533576888376337251 // 1/(2Pi)
#define M_SQRT2    1.41421356237309504880 


namespace SDE_Framework::Solvers
{
        namespace helper
        {
            template<typename generator>
            generator createSeededGenerator()
            {
#ifdef USE_PCG_RANDOM
                // Seed with a real random value, if available
                pcg_extras::seed_seq_from<std::random_device> seq;
#else
                std::random_device rd;
                std::array<std::random_device::result_type, generator::state_size> seed_data;
                std::generate(seed_data.begin(), seed_data.end(), [&]() {return rd(); });
                std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
#endif
                return generator{ seq };
            }

            template<typename generator>
            generator createSeededGenerator([[maybe_unused]] std::random_device& rd)
            {
#ifdef USE_PCG_RANDOM
                // Seed with a real random value, if available
                pcg_extras::seed_seq_from<std::random_device> seq;
#else
                std::array<std::random_device::result_type, generator::state_size> seed_data;
                std::generate(seed_data.begin(), seed_data.end(), [&]() {return rd(); });
                std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
#endif
                return generator{ seq };
            }

            /***********************Template Recursion Helpers*******************************/
                    
            template <typename prec, unsigned int r>
            inline typename std::enable_if_t<(r >= 2u), prec> pr()
            {
                //TODO: Rewrite using constexpr if as soon as possible with clang, clang-cl and visual studio
                return (1.0 / (static_cast<prec>(r*r)) + pr<prec, (r - 1u)>());
            }

            template <typename prec, unsigned int r>
            inline typename std::enable_if_t<(r == 1u), prec> pr()
            {
                return (1.0);
            }

            template <typename prec, unsigned int r>
            inline std::enable_if_t<(r == 0u), prec> pr()
            {
                return (0.0);
            }
                        
            /// <summary>	Calculates 1/12-1/2pi^2*Sum(1/r^2,1,p). </summary>
            template <typename prec, unsigned int p = 0>
            inline prec sqrtpp()
            {
                //TODO: Use sqrtf and sqrtl for other floating point types!
                return std::sqrt(0.08333333333333333333333 - M_1_DIV_2_PISQR*pr<prec, p>());
            }
        }

        //See Kloeden and Platen Numerical Solution of Stochastic Differential Equations Page 202 Chapter 5.8 Approximate Multiple Stratonovich Integrals

        /*************************DoubleNoiseMatrix*****************************/ //General Case
        template<typename prec, int p, unsigned int dim, typename generator = std::mt19937_64 >
        class DoubleNoiseMatrix
        {
            static_assert(p > 0, "P should be greater than 0; did not choose spec. Class");
            static_assert(dim < std::numeric_limits<int>::max());
        public:
            using Generator = generator;

        protected:
            DoubleNoiseMatrix() {};
        private:
            const prec m_dt;
            const prec m_sqrtdt;
            const prec m_sqrtpp;
            std::array<Generator, dim> mu_j;
            //std::array<Generator, dim*p> eta_jr; // n mit verlaengerung
            //std::array<Generator, dim*p> zeta_jr; // C mit schnoerkeln
            std::array<std::array<Generator, (unsigned int)p>, dim> eta_jr; // n mit verlaengerung
            std::array<std::array<Generator, (unsigned int)p>, dim> zeta_jr; // C mit schnoerkeln


#ifdef USE_BOOST_RANDOM
            boost::random::normal_distribution<prec> m_distribution;
#else
            std::normal_distribution<prec> m_distribution;
#endif	

            template <unsigned int r>
            inline typename std::enable_if_t<(r >= 2u), prec> rsum(const unsigned int j1, const prec dWj1, const unsigned int j2, const prec dWj2)
            {
                return (1.0 / r*(m_distribution(zeta_jr[j1][(r - 1)])*(M_SQRT2*dWj2 - m_distribution(zeta_jr[j2][(r - 1)])) 
                    - m_distribution(zeta_jr[j2][(r - 1)])*(M_SQRT2*dWj1 - m_distribution(zeta_jr[j1][(r - 1)]))) + rsum<r - 1u>(j1, dWj1, j2, dWj2));
            }

            template <unsigned int r>
            inline typename std::enable_if_t<(r == 1u), prec> rsum(const unsigned int j1, const prec dWj1, const unsigned int j2, const prec dWj2)
            {
                return (m_distribution(zeta_jr[j1][0])*(M_SQRT2*dWj2 - m_distribution(zeta_jr[j2][0])) 
                    - m_distribution(zeta_jr[j2][0])*(M_SQRT2*dWj1 - m_distribution(zeta_jr[j1][0])));
            }

            /***********************************************************************/
        public:
            /**********************************************************************************/
            DoubleNoiseMatrix(const std::size_t& NumberOfInit, const prec timestep) 
                : m_dt(timestep), m_sqrtdt(sqrt(timestep)), m_sqrtpp(helper::sqrtpp<prec,p>())
            {
                std::random_device rd;
                for (unsigned int j = dim; j--;)
                {
                    mu_j[j] = helper::createSeededGenerator<generator>(rd);
                    mu_j[j].discard(NumberOfInit);
                    for (unsigned int r = p; r--;)
                    {
                        //this works put jumps in memory
                        //const auto tmp = j + r*dim;

                        eta_jr[j][r] = helper::createSeededGenerator<generator>(rd);
                        eta_jr[j][r].discard(NumberOfInit);

                        zeta_jr[j][r] = helper::createSeededGenerator<generator>(rd);
                        zeta_jr[j][r].discard(NumberOfInit);
                    };
                };
#ifdef USE_BOOST_RANDOM
                m_distribution = boost::random::normal_distribution<prec>{ 0, m_sqrtdt };
#else
                m_distribution = std::normal_distribution<prec>{ 0, m_sqrtdt };
#endif //USE_BOOST_RANDOM
            };

            /********************************************************************************/
            inline Eigen::Matrix<prec, (int)dim, (int)dim> getNoiseMatrix(const Eigen::Matrix<prec, (int)dim, 1> &dWi)
            {
                Eigen::Matrix<prec, (int)dim, (int)dim> J_j1j2{ Eigen::Matrix<prec, (int)dim, (int)dim>::Zero() };

                for (int j1 = (int)dim; j1--;) // be aware of the fact that j1 is decremented before it is used!
                {
                    prec tmp = 0.0;
                    for (int j2 = j1; j2--;) //makes sure that j2 is always smaller than j1
                    {
                        //symmetric part

                        J_j1j2(j1, j2) = 0.5*dWi(j1, 0)*dWi(j2, 0);
                        J_j1j2(j2, j1) = J_j1j2(j1, j2);

                        tmp = 0.0;

                        //asymmetric part
                        tmp -= rsum<p>(j1, dWi(j1, 0), j2, dWi(j2, 0));
                        //for (unsigned int r = p; r--;)
                        //{
                        //	// r+1 due to r--;
                        //	//tmp += 1.0 / (r + 1.0)*(m_distribution(zeta_jr[r + j1*p])*(M_SQRT2*dWi(j2, 0) + m_distribution(zeta_jr[r + p*j2])) - m_distribution(zeta_jr[r + p*j2])*(M_SQRT2*dWi(j1, 0) + m_distribution(zeta_jr[r + p*j1])));
                        //	tmp += 1.0 / (r + 1.0)*(m_distribution(zeta_jr[j1][r])*(M_SQRT2*dWi(j2, 0) + m_distribution(zeta_jr[j2][r])) 
                        //		- m_distribution(zeta_jr[j2][r])*(M_SQRT2*dWi(j1, 0) + m_distribution(zeta_jr[j1][r])));						
                        //};
                        tmp *= M_1_DIV_2PI;
                        
                        tmp += m_sqrtpp*(m_distribution(mu_j[j1])*dWi(j2, 0) - m_distribution(mu_j[j2])*dWi(j1, 0)); //Double Checked! This term seems to be correct!

                        J_j1j2(j1, j2) += tmp;
                        J_j1j2(j2, j1) -= tmp;

                    };
                    //diagonal part (in Stratonovich intepretation)
                    J_j1j2(j1, j1) = 0.5*dWi(j1, 0)*dWi(j1, 0);
                };
                return J_j1j2;
            };
        };
        /**************************Spezialisierung f�r p=0*********************************************/
        template<typename prec, unsigned int dim, typename generator>
        class DoubleNoiseMatrix<prec, 0, dim, generator >
        {
            static_assert(dim < std::numeric_limits<int>::max());
        protected:
            DoubleNoiseMatrix() {};

        private:
            const prec m_dt;
            const prec m_sqrtdt;
            const prec m_sqrtpp;
            std::array<generator, dim> mu_j;

#ifdef USE_BOOST_RANDOM
            boost::random::normal_distribution<prec> m_distribution;
#else
            std::normal_distribution<prec> m_distribution;
#endif	

            Eigen::Matrix<prec, (int)dim, (int)dim> I_j1j2{ Eigen::Matrix<prec, (int)dim, (int)dim>::Zero() };


        public:
            DoubleNoiseMatrix(std::size_t NumberOfInit, prec timestep) 
                : m_dt(timestep), m_sqrtdt(std::sqrt(timestep)), m_sqrtpp(helper::sqrtpp<prec,0>())
            {
                std::random_device rd;
                for (unsigned int j = dim; j--;)
                {
                    mu_j[j] = helper::createSeededGenerator<generator>(rd);
                    mu_j[j].discard(NumberOfInit);
                };

#ifdef USE_BOOST_RANDOM
                m_distribution = boost::random::normal_distribution<prec>{ 0, this->m_sqrtdt };
#else
                m_distribution = std::normal_distribution<prec>{ 0, this->m_sqrtdt };
#endif //USE_BOOST_RANDOM
            };

            inline Eigen::Matrix<prec, (int)dim, (int)dim> getNoiseMatrix(const Eigen::Matrix<prec, (int)dim, 1> &dWi)
            {
                Eigen::Matrix<prec, (int)dim, (int)dim> J_j1j2;

                for (int j1 = dim; j1--;) // be aware of the fact that j1 is decremented before it is used!
                {
                    prec tmp = 0.0;
                    for (int j2 = j1; j2--;) //makes sure that j2 is always smaller than j1
                    {
                        //symmetric part

                        J_j1j2(j1, j2) = 0.5*dWi(j1, 0)*dWi(j2, 0);
                        J_j1j2(j2, j1) = J_j1j2(j1, j2);

                        //asymmetric part
                        tmp = m_sqrtpp*(m_distribution(mu_j[j1])*dWi(j2, 0) - m_distribution(mu_j[j2])*dWi(j1, 0));
                        J_j1j2(j1, j2) += tmp;
                        J_j1j2(j2, j1) -= tmp;

                    };
                    //diagonal part (in Stratonovich intepretation)
                    J_j1j2(j1, j1) = 0.5*dWi(j1, 0)*dWi(j1, 0);
                };

                return J_j1j2;
            };

        };

        /**************************Spezialisierung f�r p=-1*********************************************/
        template<typename prec, unsigned int dim, typename generator>
        class DoubleNoiseMatrix<prec, -1, dim, generator>
        {
            static_assert(dim < std::numeric_limits<int>::max());
        private:
            Eigen::Matrix<prec, (int)dim, (int)dim> I_j1j2{ Eigen::Matrix<prec, (int)dim, (int)dim>::Zero() };

        public:
            DoubleNoiseMatrix(const int /*NumberOfInit*/, const prec /*timestep*/) {};

            inline Eigen::Matrix<prec, (int)dim, (int)dim> calculateNoiseMatrix(const Eigen::Matrix<prec, (int)dim, 1> &dWi)
            {
                for (int i = dim; i--;)
                {
                    for (int j = i; j--;)
                    {
                        I_j1j2(i, j) = 0.5 * dWi(i, 0)* dWi(j, 0);
                        I_j1j2(j, i) = I_j1j2(i, j);
                    };
                    I_j1j2(i, i) = 0.5 * dWi(i, 0)* dWi(i, 0);
                };
                return I_j1j2;
            };

            inline Eigen::Matrix<prec, (int)dim, (int)dim> getNoiseMatrix(const Eigen::Matrix<prec, (int)dim, 1> &dWi)
            {
                return this->calculateNoiseMatrix(dWi);
            };
        };
}
#endif //_DoubleNoiseMatrix_H_
