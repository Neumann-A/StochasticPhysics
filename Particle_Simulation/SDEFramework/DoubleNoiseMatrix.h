/* DoubleNoiseMatrix
* Calculates the Noise Matrix which is necessary for explicit Solvers with strong order 1 and higher
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#include "General/Setup.h"
#define M_1_DIV_2_PISQR 0.05066059182116888572193973160486 // 1/(2 Pi^2)
#define M_1_DIV_2PI 0.15915494309189533576888376337251 // 1/(2Pi)
#pragma once

#ifndef _DoubleNoiseMatrix_H_
#define _DoubleNoiseMatrix_H_

/*************************DoubleNoiseMatrix*****************************/ //General Case
	template<typename prec, int p, unsigned int dim, typename generator >
	class DoubleNoiseMatrix
	{
		static_assert(p > 0, "P should be greater than 0; did not choose spec. Class");
	public:
		typedef generator	Generator;

	protected:
		DoubleNoiseMatrix() {};
	private:
		const prec m_dt;
		const prec m_sqrtdt;
		const prec m_sqrtrhop;
		std::array<generator, dim> mu_j;
		std::array<generator, dim*p> eta_jr; // n mit verl�ngerung
		std::array<generator, dim*p> zeta_jr; // C mit schn�rkeln

#ifdef USE_BOOST
		boost::random::normal_distribution<prec> m_distribution;
#else
		std::normal_distribution<prec> m_distribution;
#endif	

		void initGenerators(const int NumberOfInit)
		{

			for (generator& gen : this->mu_j)
			{
				this->initGenerator(gen, NumberOfInit);
			};

			for (generator& gen : this->eta_jr)
			{
				this->initGenerator(gen, NumberOfInit);
			};

			for (generator& gen : this->zeta_jr)
			{
				this->initGenerator(gen, NumberOfInit);
			};

		};

		inline void initGenerator(generator gen, const int NumberOfInit)
		{
			volatile prec &&tmp = 0;
			for (int i = NumberOfInit; i--;)
			{
				tmp = this->m_distribution(gen);
			};
			tmp;
		};

		/***********************Template Recursion Helpers*******************************/

		template <unsigned int n>
		inline static typename std::enable_if_t<(n >= 2u), prec> helperrhop()
		{
			return (1 / (n*n) + helperrhop<(n-1u)>());
		};

		template <unsigned int n>
		inline static typename std::enable_if_t<(n == 1u), prec> helperrhop()
		{
			return (1.0);
		};

		//Should never be reach but just in case
		template <unsigned int n>
		inline static typename std::enable_if_t<(n == 0u), prec> helperrhop()
		{
			return (0.0);
		};

		template <unsigned int n = 0>
		inline static typename std::enable_if_t<(n >= 0),prec> sqrtrhop()
		{
			//std::cout << 1 / 12 << " " << M_1_DIV_2_PISQR  << std::endl;
			return sqrt(0.8333333333333333333333 - M_1_DIV_2_PISQR*helperrhop<n>() );
		};

		template <unsigned int r>
		inline typename std::enable_if_t<(r >= 2u), prec> rsum(const unsigned int j1, const prec dWj1, const unsigned int j2, const prec dWj2)
		{
			//thats right but stupid memory access (makes no difference)
			//return (1 / r*(this->m_distribution(this->zeta_jr[j1+r*dim])*(M_SQRT2*dWj2 + this->m_distribution(this->zeta_jr[j2 + r*dim])) - this->m_distribution(this->zeta_jr[j2 + r*dim])*(M_SQRT2*dWj1 + this->m_distribution(this->zeta_jr[j1 + r*dim]))) + this->rsum<r - 1>(dWj1, dWj2));

			// better memory access since we are summing over r; so that the pointer does not shift that much
			return (1 / r*(this->m_distribution(this->zeta_jr[(r - 1) + j1*p])*(M_SQRT2*dWj2 + this->m_distribution(this->zeta_jr[(r - 1) + p*j2])) - this->m_distribution(this->zeta_jr[(r - 1) + p*j2])*(M_SQRT2*dWj1 + this->m_distribution(this->zeta_jr[(r - 1) + p*j1]))) + rsum<r-1u>(j1, dWj1, j2, dWj2));
		};

		template <unsigned int r>
		inline typename std::enable_if_t<(r == 1u), prec> rsum(const unsigned int j1, const prec dWj1, const unsigned int j2, const prec dWj2)
		{
			return (this->m_distribution(this->zeta_jr[p*j1])*(M_SQRT2*dWj2 + this->m_distribution(this->zeta_jr[p*j2])) - this->m_distribution(this->zeta_jr[p*j2])*(M_SQRT2*dWj1 + this->m_distribution(this->zeta_jr[p*j1])));
		};

		//Should never be reached but just in case
		template <unsigned int r>
		inline typename std::enable_if_t<(r == 0u), prec> rsum(const unsigned int /*j1*/, const prec /*dWj1*/, const unsigned int /*j2*/, const prec /*dWj2*/)
		{
			return 0;
		};

		/***********************************************************************/
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		/**********************************************************************************/
		DoubleNoiseMatrix(const int NumberOfInit, const prec timestep) : m_dt(timestep), m_sqrtdt(sqrt(timestep)), m_sqrtrhop(DoubleNoiseMatrix<prec, p, dim, generator>::sqrtrhop<p>())
		{
			std::random_device rd;
			unsigned int tmp = 0;
			for (unsigned int j = dim; j--;)
			{
				this->mu_j[j] = generator(rd());
				for (unsigned int r = p; r--;)
				{
					//this works put jumps in memory
					tmp = j + r*dim;
					this->eta_jr[tmp] = generator(rd());
					this->zeta_jr[tmp] = generator(rd());
				};
			};
#ifdef USE_BOOST
			this->m_distribution = boost::random::normal_distribution<prec>{ 0, this->m_sqrtdt };
#else
			this->m_distribution = std::normal_distribution<prec>{ 0, this->m_sqrtdt };
#endif //USE_BOOST

			this->initGenerators(NumberOfInit);
		};
		/********************************************************************************/
		~DoubleNoiseMatrix() {};
		/********************************************************************************/
		inline Eigen::Matrix<prec, dim, dim> getNoiseMatrix(const Eigen::Matrix<prec, dim, 1> &dWi)
		{
			Eigen::Matrix<prec, dim, dim> J_j1j2;

			//prec tmp2 = 0;

			for (int j1 = dim; j1--;) // be aware of the fact that j1 is decremented before it is used!
			{
				prec tmp = 0.0;
				for (int j2 = j1; j2--;) //makes sure that j2 is always smaller than j1
				{
					//symmetric part

					J_j1j2(j1, j2) = 0.5*dWi(j1, 0)*dWi(j2, 0);
					J_j1j2(j2, j1) = J_j1j2(j1, j2);

					tmp = 0.0;
					////asymmetric part
					//for (unsigned int r = p; r--;)
					//{
					//	tmp += 1.0 / (r + 1.0)*(this->m_distribution(this->zeta_jr[r + j1*p])*(M_SQRT2*dWi(j2, 0) + this->m_distribution(this->zeta_jr[r + p*j2])) - this->m_distribution(this->zeta_jr[r + p*j2])*(M_SQRT2*dWi(j1, 0) + this->m_distribution(this->zeta_jr[r + p*j1])));
					//};
					//tmp *= M_1_DIV_2PI;
					//tmp += this->m_sqrtrhop*(this->m_distribution(this->mu_j[j1])*dWi(j2, 0) - this->m_distribution(this->mu_j[j2])*dWi(j1, 0));

					//J_j1j2(j1, j2) += tmp;
					//J_j1j2(j2, j1) -= tmp;

				};
				//diagonal part (in Stratonovich intepretation)
				J_j1j2(j1, j1) = 0.5*dWi(j1, 0)*dWi(j1, 0);
			};
			return J_j1j2;
		};
	};
	/**************************Spezialisierung f�r p=0*********************************************/
	template<typename prec,unsigned int dim, typename generator>
	class DoubleNoiseMatrix<prec, 0, dim, generator >
	{
	protected:
		DoubleNoiseMatrix() {};

	private:
		const prec m_dt;
		const prec m_sqrtdt;
		const prec m_sqrtrhop;
		std::array<generator, dim> mu_j;

#ifdef USE_BOOST
		boost::random::normal_distribution<prec> m_distribution;
#else
		std::normal_distribution<prec> m_distribution;
#endif	

		Eigen::Matrix<prec, dim, dim> I_j1j2{ Eigen::Matrix<prec, dim, dim>::Zero() };

		void initGenerators(const int NumberOfInit)
		{
			for (generator &gen : this->mu_j)
			{
				this->initGenerator(gen, NumberOfInit);
			};
		};

		inline void initGenerator(generator gen, const int NumberOfInit)
		{
			volatile prec &&tmp = 0;
			for (int i = NumberOfInit; i--;)
			{
				tmp = this->m_distribution(gen);
			};
			tmp;
		};
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		DoubleNoiseMatrix(const int NumberOfInit, const prec timestep) : m_dt(timestep), m_sqrtdt(sqrt(timestep)), m_sqrtrhop(0.912870929175276855761616304668)
		{
			std::random_device rd;
			for (unsigned int j = dim; j--;)
			{
				this->mu_j[j] = generator(rd());
			};

#ifdef USE_BOOST
			this->m_distribution = boost::random::normal_distribution<prec>{ 0, this->m_sqrtdt };
#else
			this->m_distribution = std::normal_distribution<prec>{ 0, this->m_sqrtdt };
#endif //USE_BOOST

			this->initGenerators(NumberOfInit);
		};

		~DoubleNoiseMatrix() {};

		inline Eigen::Matrix<prec, dim, dim> getNoiseMatrix(const Eigen::Matrix<prec, dim, 1> &dWi)
		{
			Eigen::Matrix<prec, dim, dim> J_j1j2;

			for (int j1 = dim; j1--;) // be aware of the fact that j1 is decremented before it is used!
			{
				prec tmp = 0.0;
				for (int j2 = j1; j2--;) //makes sure that j2 is always smaller than j1
				{
					//symmetric part

					J_j1j2(j1, j2) = 0.5*dWi(j1, 0)*dWi(j2, 0);
					J_j1j2(j2, j1) = J_j1j2(j1, j2);

					//asymmetric part
					tmp = this->m_sqrtrhop*(this->m_distribution(this->mu_j[j1])*dWi(j2, 0) - this->m_distribution(this->mu_j[j2])*dWi(j1, 0));
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
	private:

		Eigen::Matrix<prec, dim, dim> I_j1j2;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		DoubleNoiseMatrix() {};
		DoubleNoiseMatrix(const int /*NumberOfInit*/, const prec /*timestep*/) {};
		~DoubleNoiseMatrix() {};

		inline Eigen::Matrix<prec, dim, dim> calculateNoiseMatrix(const Eigen::Matrix<prec, dim, 1> &dWi)
		{
			//Eigen::Matrix<prec, dim, dim> I_j1j2

			for (int i = dim; i--;)
			{
				for (int j = i; j--;)
				{
					this->I_j1j2(i, j) = 0.5 * dWi(i, 0)* dWi(j, 0);
					this->I_j1j2(j, i) = this->I_j1j2(i, j);
					//std::cout << "Calc Matrix Element" << i << "," << j << std::endl;
				};
				this->I_j1j2(i, i) = 0.5 * dWi(i, 0)* dWi(i, 0);
			};
			return this->I_j1j2;
		};

		inline Eigen::Matrix<prec, dim, dim> getNoiseMatrix(const Eigen::Matrix<prec, dim, 1> &dWi)
		{
			return this->calculateNoiseMatrix(dWi);
		};
	};

	
#endif //_DoubleNoiseMatrix_H_