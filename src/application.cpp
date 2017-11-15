#include "application.h"

#include <unordered_map>
#include "laser.h"
#include "radar.h"

#include "kalman_filter.h"

namespace {
  template<typename M>
  State process(std::istringstream& iss, State state) {
    typename M::Package p;
    iss >> p;
    return kalman_filter::update<Model, M>(std::move(p), std::move(state));
  }

  const std::unordered_map<char, decltype(&update)> registry {
    {Laser::Tag, &process<Laser>},
    {Radar::Tag, &process<Radar>},
  };
}

State update(std::istringstream& iss, State state) {
  try {
    char type = 0;
    iss >> type;
    return registry.at(type)(iss, state);
  } catch(std::out_of_range& ) {
    throw std::runtime_error("unexpected sensor type");
  }
}
