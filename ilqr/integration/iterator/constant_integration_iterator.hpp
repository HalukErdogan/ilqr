#ifndef ILQR_INTEGRATION_ITERATOR_CONSTANT_INTEGRATION_ITERATOR_HPP
#define ILQR_INTEGRATION_ITERATOR_CONSTANT_INTEGRATION_ITERATOR_HPP

#include "ilqr/integration/iterator/integration_iterator.hpp"
#include "ilqr/integration/stepper/integration_stepper.hpp"

namespace ilqr {
namespace integration {
namespace iterator {

template <std::size_t N, std::size_t M>
class ConstantIntegrationIterator : public IntegrationIterator<N, M> {
    public:
    void Next(const StepperPtr& stepper, const SystemPtr &system, const State& state, const Control& control,
              const Time& time, const Time& delta_time, State& result) const override {
        stepper->Step(system, state, control, time, delta_time, result);
    }
};

}  // namespace iterator
}  // namespace integration
}  // namespace ilqr
#endif  // ILQR_INTEGRATION_ITERATOR_CONSTANT_INTEGRATION_ITERATOR_HPP