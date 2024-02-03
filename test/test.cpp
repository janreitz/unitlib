#include "../include/unitlib.h"

using namespace Unitlib;

int main(int, char **)
{

  // Test for basic instantiation and value retrieval
  constexpr Meter<double> oneMeter(1.0);
  constexpr Kilogram<double> oneKilogram(1.0);
  constexpr Second<double> oneSecond(1.0);
  constexpr Ampere<double> oneAmpere(1.0);
  constexpr Kelvin<double> oneKelvin(1.0);
  constexpr Mole<double> oneMole(1.0);
  constexpr Candela<double> oneCandela(1.0);
  static_assert(oneMeter.getValue() == 1.0, "One meter should have a value of 1.0");

  // Test for dimensional analysis - multiplication
  constexpr Meter<double> length(2.0);
  constexpr Unit<Area, double> squareMeter = length * length;
  static_assert(squareMeter.getValue() == 4.0, "Area calculation failed");

  // Test for unit conversion
  constexpr Kilometer<double> oneMeterInKilometers = oneMeter;
  static_assert(oneMeterInKilometers.getValue() == 0.001, "Meter to Kilometer conversion failed");

  // Test for arithmetic operations
  constexpr Meter<double> twoMeters = oneMeter + oneMeter;
  static_assert(twoMeters.getValue() == 2.0, "Addition failed");

  return 0;
}
