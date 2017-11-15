#include "catch.hpp"
#include "../src/filter.h"

TEST_CASE("Filter<Deny>", "[filter]") {
  Filter<int, Deny> f;
  REQUIRE(f(42) == false);
  f.add(44);
  REQUIRE(f(42) == false);
  REQUIRE(f(44) == true);
}

TEST_CASE("Filter<Allow>", "[filter]") {
  Filter<int, Allow> f;
  REQUIRE(f(42) == true);
  f.add(44);
  REQUIRE(f(42) == true);
  REQUIRE(f(44) == false);
}
