#include "../include/unitlib.h"
#include <cstdint>
#include <type_traits>

using namespace Unitlib;
using namespace Unitlib::Literals;

int main(int, char**)
{
  // Test for basic instantiation and value retrieval
  constexpr Meter<double> meter(1.0);
  constexpr Kilogram<double> kilogram(1.0);
  constexpr Second<double> second(1.0);
  constexpr Ampere<double> ampere(1.0);
  constexpr Kelvin<double> kelvin(1.0);
  constexpr Mole<double> mole(1.0);
  constexpr Candela<double> candela(1.0);
  static_assert(meter.getValue() == 1.0, "One meter should have a value of 1.0");

  // Test for dimensional analysis - multiplication
  // constexpr Newton<double> newton = kilogram * 2.0 * (meter * 10.0 / (second * 2.0 * second * 2.0));
  constexpr Newton<double> newton = 2.0 * kilogram * (10.0 * meter / (2.0 * second * 2.0 * second));
  static_assert(newton.getValue() == 5.0, "Calculation of force in newtons failed");

  constexpr Joule<double> energy = newton * meter;
  static_assert(energy.getValue() == 5.0, "Joule calculation failed");

  constexpr Watt<double> power = energy / second;
  static_assert(power.getValue() == 5.0, "Watt calculation failed");

  constexpr Pascal<double> pressure = newton / (meter * meter);
  static_assert(pressure.getValue() == 5.0, "Pascal calculation failed");

  constexpr Coulomb<double> charge = ampere * second;
  static_assert(charge.getValue() == 1.0, "Coulomb calculation failed");

  constexpr Volt<double> voltage = power / ampere;
  static_assert(voltage.getValue() == 5.0, "Volt calculation failed");

  // Test for unit conversion
  constexpr Meter<double> meter1(500.0);
  constexpr Kilometer<double> meter2(1.0);
  constexpr Meter<double> totalmeter = meter1 + meter2;
  static_assert(totalmeter.getValue() == 1500.0, "Adding meters and kilometers failed");

  // Test for arithmetic operations

  // Test literals
  constexpr auto acceleration = 1.0_m / (1.0_s * 1.0_s);
  static_assert(acceleration.getValue() == 1.0, "Floating point literal failed");

  constexpr auto acceleration_int = 1_m / (1_s * 1_s);
  static_assert(std::is_same_v<decltype(acceleration_int)::VT, int64_t>, "Value Type of integer literal computation not as expected");
  static_assert(std::is_same_v<decltype(acceleration_int)::D, Acceleration>, "Dimension of integer literal computation not as expected");
  static_assert(acceleration_int.getValue() == 1, "Integer literals failed");
  return 0;
}
