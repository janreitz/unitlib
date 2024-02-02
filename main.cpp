#include <iostream>
#include <ratio>
#include <string>
#include <type_traits>

template<typename Length = std::ratio<0>,
  typename Mass = std::ratio<0>,
  typename Time = std::ratio<0>,
  typename ElectricCurrent = std::ratio<0>,
  typename ThermodynamicTemperature = std::ratio<0>,
  typename AmountOfSubstance = std::ratio<0>,
  typename LuminousIntensity = std::ratio<0>>
struct Dimension
{
  // Type aliases for template parameters makes them accessible later on
  using L = Length;
  using M = Mass;
  using T = Time;
  using I = ElectricCurrent;
  using Th = ThermodynamicTemperature;
  using N = AmountOfSubstance;
  using J = LuminousIntensity;

  // Operator overload for multiplying dimensions
  template<typename Other> constexpr auto operator*(const Other &) const
  {
    return Dimension<std::ratio_add<L, typename Other::L>,
      std::ratio_add<M, typename Other::M>,
      std::ratio_add<T, typename Other::T>,
      std::ratio_add<I, typename Other::I>,
      std::ratio_add<Th, typename Other::Th>,
      std::ratio_add<N, typename Other::N>,
      std::ratio_add<J, typename Other::J>>{};
  }

  // Operator overload for dividing dimensions
  template<typename Other> constexpr auto operator/(const Other &) const
  {
    return Dimension<std::ratio_subtract<L, typename Other::L>,
      std::ratio_subtract<M, typename Other::M>,
      std::ratio_subtract<T, typename Other::T>,
      std::ratio_subtract<I, typename Other::I>,
      std::ratio_subtract<Th, typename Other::Th>,
      std::ratio_subtract<N, typename Other::N>,
      std::ratio_subtract<J, typename Other::J>>{};
  }
};

// Template alias for multiplying dimensions
template<typename D1, typename D2>
using MultiplyDimensions = Dimension<std::ratio_add<typename D1::L, typename D2::L>,
  std::ratio_add<typename D1::M, typename D2::M>,
  std::ratio_add<typename D1::T, typename D2::T>,
  std::ratio_add<typename D1::I, typename D2::I>,
  std::ratio_add<typename D1::Th, typename D2::Th>,
  std::ratio_add<typename D1::N, typename D2::N>,
  std::ratio_add<typename D1::J, typename D2::J>>;

// Template alias for dividing dimensions
template<typename D1, typename D2>
using DivideDimensions = Dimension<std::ratio_subtract<typename D1::L, typename D2::L>,
  std::ratio_subtract<typename D1::M, typename D2::M>,
  std::ratio_subtract<typename D1::T, typename D2::T>,
  std::ratio_subtract<typename D1::I, typename D2::I>,
  std::ratio_subtract<typename D1::Th, typename D2::Th>,
  std::ratio_subtract<typename D1::N, typename D2::N>,
  std::ratio_subtract<typename D1::J, typename D2::J>>;

using Length = Dimension<std::ratio<1>>;
using Area = MultiplyDimensions<Length, Length>;
using Time = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using Frequency = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<-1>>;
using Velocity = DivideDimensions<Length, Time>;

template<typename Dimension, typename ValueType, typename ScalingFactor = std::ratio<1>> class Unit
{
  ValueType value_;

public:
  // Publicly expose the Dimension and ScalingFactor for use in conversions and checks
  using D = Dimension;
  using VT = ValueType;
  using SF = ScalingFactor;

  // Constructor
  explicit constexpr Unit(const ValueType &value) : value_(value) {}

  // Getter for the unit value
  constexpr ValueType getValue() const { return value_; }

  constexpr operator ValueType() const { return getValue(); }

  // Conversion function to another unit within the same dimension
  template<typename OtherUnit> constexpr operator OtherUnit() const
  {
    static_assert(std::is_same<D, typename OtherUnit::D>::value, "Incompatible dimensions for conversion.");

    // Convert from this unit to the base unit, then from the base unit to the target unit
    return OtherUnit(value_ * (ScalingFactor::num / static_cast<ValueType>(ScalingFactor::den))
                     * (OtherUnit::SF::den / static_cast<ValueType>(OtherUnit::SF::num)));
  }

  // Operator overload for multiplying units
  template<typename OtherUnit> constexpr auto operator*(const OtherUnit &other) const
  {
    static_assert(std::is_same<ValueType, typename OtherUnit::VT>::value, "Incompatible ValueTypes");
    using NewDimension = MultiplyDimensions<D, typename OtherUnit::D>;
    using NewScalingFactor = std::ratio_multiply<SF, typename OtherUnit::SF>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ value_ * other.getValue() };
  }

  // Operator overload for multiplying units
  template<typename OtherUnit> constexpr auto operator/(const OtherUnit &other) const
  {
    static_assert(std::is_same<ValueType, typename OtherUnit::VT>::value, "Incompatible ValueTypes");
    using NewDimension = DivideDimensions<D, typename OtherUnit::D>;
    using NewScalingFactor = std::ratio_divide<SF, typename OtherUnit::SF>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ value_ / other.getValue() };
  }
};

// Assuming Length and Time dimensions are defined as before
using Meter = Unit<Length, double>;// Base unit of Length
using Kilometer = Unit<Length, double, std::ratio<1000>>;// 1000 meters in a kilometer

using Second = Unit<Time, double>;// Base unit of Time
using Millisecond = Unit<Time, double, std::ratio<1, 1000>>;// 1000 milliseconds in a second
using Minute = Unit<Time, double, std::ratio<60>>;
using Hour = Unit<Time, double, std::ratio<3600>>;
using Hertz = Unit<Frequency, double>;

int main(int, char **)
{
  constexpr Meter distance(100);// 100 meters
  constexpr Kilometer distance_km = distance;// Convert to kilometers

  std::cout << "Kilometers: " << double(distance_km) << '\n';

  constexpr Second time(30);
  constexpr Millisecond time_ms = time;
  constexpr Hour time_h = time;
  std::cout << double(time_ms) << '\n';

  auto area = distance * distance;
  std::cout << "Square meters" << double(area) << '\n';

  constexpr Unit<DivideDimensions<Length, Time>, double> meters_per_second = distance_km / time_h;

  auto kilometers_per_hour = distance_km / time_h;

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(meters_per_second) << "[m/s]"
            << '\n';

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(kilometers_per_hour)
            << "[km/h]" << '\n';


  return 0;
}
