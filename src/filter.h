#pragma once

#include <unordered_set>

struct Allow { static constexpr bool value = true; };
struct Deny { static constexpr bool value = false; };
  
template<typename T, typename Policy>
class Filter {
public:
  Filter& add(T value) {
    chain_.emplace(std::move(value));
    return *this;
  }

  bool operator()(const T& value) const {
    return chain_.count(value) == 0 ? Policy::value : !Policy::value;
  }
  
private:
  std::unordered_set<T> chain_;
};
