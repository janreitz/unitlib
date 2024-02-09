#include "../include/unitlib.h"
#include <cmath>
#include <cstdint>
#include <ratio>

using namespace Unitlib;
using namespace Unitlib::Literals;

template<typename U, typename L, typename v>
requires HasDimension<U, Velocity> && HasDimension<L, Length> && HasDimension<v, KinematicViscosity>
constexpr auto reynolds_number(const U &flow_speed, const L &characteristic_length, const v &kinematic_viscosity)
{
  return flow_speed * characteristic_length / kinematic_viscosity;
}

int main(int, char **)
{
  // Test for basic instantiation and value retrieval
  constexpr Meter<double> meter(1.0);
  constexpr Kilogram<double> kilogram(1.0);
  constexpr Second<double> second(1.0);
  constexpr Ampere<double> ampere(1.0);
  constexpr Kelvin<double> kelvin(1.0);
  constexpr Mole<double> mole(1.0);
  constexpr Candela<double> candela(1.0);
  static_assert(meter.get_value() == 1.0, "One meter should have a value of 1.0");

  using Kilohertz = Unit<Frequency, double, std::kilo>;
  constexpr Kilohertz kilohertz(1.0);
  static_assert(kilohertz.get_base_value() == 1000.0, "One kilohertz should have a base value of 1000.0");

  // Test for dimensional analysis - multiplication
  constexpr Newton<double> newton = 2.0 * kilogram * (10.0 * meter / (2.0 * second * 2.0 * second));
  static_assert(newton.get_value() == 5.0, "Calculation of force in newtons failed");

  constexpr Joule<double> energy = newton * meter;
  static_assert(energy.get_value() == 5.0, "Joule calculation failed");

  constexpr Watt<double> power = energy / second;
  static_assert(power.get_value() == 5.0, "Watt calculation failed");

  constexpr Pascal<double> pressure = newton / (meter * meter);
  static_assert(pressure.get_value() == 5.0, "Pascal calculation failed");

  constexpr Coulomb<double> charge = ampere * second;
  static_assert(charge.get_value() == 1.0, "Coulomb calculation failed");

  constexpr Volt<double> voltage = power / ampere;
  static_assert(voltage.get_value() == 5.0, "Volt calculation failed");

  // Test for unit conversion
  constexpr Meter<double> meter1(500.0);
  constexpr Kilometer<double> meter2(1.0);
  constexpr Meter<double> totalmeter = meter1 + meter2;
  static_assert(totalmeter.get_value() == 1500.0, "Adding meters and kilometers failed");
  static_assert(Kilometer<double>(1.0).get_value_in<Meter<double>>() == 1000.0);
  static_assert(Kilometer<double>(1.0) == Meter<double>(1000.0));


  // Custom unit definition
  using Inch = Unit<Length, double, std::ratio<254, 10000>>;
  static_assert(Inch(1.0).get_value_in<Meter<double>>() == 0.0254);

  // Custom unit definition with offset
  using Fahrenheit = Unit<Temperature, double, std::ratio<5, 9>, std::ratio<45967, 100>>;
  static_assert(std::abs(DegreeCelsius<double>(23.0).get_value_in<Fahrenheit>() - 73.4) < 0.1);
  static_assert(std::abs(Fahrenheit(73.4).get_base_value() - 296.15) < 0.1);

  // Test literals
  constexpr auto acceleration = 1.0_m / (1.0_s * 1.0_s);
  static_assert(acceleration.get_value() == 1.0, "Floating point literal failed");

  constexpr auto acceleration_int = 1_m / (1_s * 1_s);
  unit_check<decltype(acceleration_int), Acceleration, int64_t>();
  static_assert(acceleration_int.get_value() == 1, "Integer literals failed");
  return 0;

  // Test concepts
  static_assert(reynolds_number(1_m / 1_s, 1_m, 1_m * 1_m / 1_s).get_value() == 1);

  // Test for comparisons
  static_assert(Meter<double>(1.0) < Meter<double>(2.0));
  static_assert(Meter<double>(1.0) <= Meter<double>(2.0));
  static_assert(Meter<double>(2.0) > Meter<double>(1.0));
  static_assert(Meter<double>(2.0) >= Meter<double>(1.0));
  static_assert(Meter<double>(1.0) == Meter<double>(1.0));
  static_assert(Meter<double>(1.0) != Meter<double>(2.0));
  static_assert(Kilometer<double>(1.0) == Meter<double>(1000.0));
  static_assert(Kilometer<double>(1.0) != Meter<double>(1001.0));
  static_assert(Kilometer<double>(1.0) < Meter<double>(1001.0));
  static_assert(Kilometer<double>(1.0) <= Meter<double>(1001.0));
}
