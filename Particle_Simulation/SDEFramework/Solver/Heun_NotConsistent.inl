///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Heun_NotConsistent.inl
//
// summary:	Heun not consistent class
///-------------------------------------------------------------------------------------------------
#pragma once

namespace SDE_Framework::Solvers
{
    template<typename problem, typename nfield>
    BASIC_ALWAYS_INLINE Heun_NotConsistent<problem, nfield>::Heun_NotConsistent(const Settings&, Problem &prob, Precision tstep)
        : GeneralSDESolver<Heun_NotConsistent<problem, nfield>, problem, nfield>(prob, std::move(tstep))
    {}

    template<typename problem, typename nfield>
    BASIC_ALWAYS_INLINE auto Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestep(const DependentType& yi, const IndependentType& xi) const noexcept //-> ResultType
    {
        return detail::FixedTimestepSelector<IsIto::value>::SelectImpl(*this, yi, xi);
    }

    template<typename problem, typename nfield>
    BASIC_ALWAYS_INLINE auto Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestepIto(const DependentType& yi, const IndependentType& xi) const noexcept -> ResultType
    {
        const auto dW = this->m_dWgen.getField();
        const auto dW2 = this->m_dWgen.getField();
        const auto ayi = (this->m_problem).getDeterministicVector(yi, xi);
        const auto byi = (this->m_problem).getStochasticMatrix(yi);
        const auto bdW = (byi*dW).eval();
        auto Support = (yi + ayi*this->m_timestep + bdW).eval();
        auto HeunStep = yi + 0.5 * ((this->m_problem).getDeterministicVector(Support, xi) + ayi)*this->m_timestep
            + 0.5*((this->m_problem).getStochasticMatrix(Support) + byi)*dW2;
        return HeunStep.eval();
    }

    template<typename problem, typename nfield>
    BASIC_ALWAYS_INLINE auto Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestepStratonovich(const DependentType& yi, const IndependentType& xi) const noexcept -> ResultType
    {
        const auto dW = this->m_dWgen.getField();
        const auto dW2 = this->m_dWgen.getField();
        const auto ayi = (this->m_problem).getDeterministicVector(yi, xi) + (this->m_problem).getDrift(yi);
        const auto byi = (this->m_problem).getStochasticMatrix(yi);
        const auto bdW = (byi*dW).eval();
        auto Support = (yi + ayi*this->m_timestep + bdW).eval();
        auto HeunStep = yi + 0.5 * ((this->m_problem).getDeterministicVector(Support, xi) + (this->m_problem).getDrift(Support) + ayi)*this->m_timestep
            + 0.5 * ((this->m_problem).getStochasticMatrix(Support) + byi)*dW2;;
        return HeunStep.eval();
    }
}
