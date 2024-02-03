#include <ratio>
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
  // Publicly expose template parameters for use in conversions and checks
  using L = Length;
  using M = Mass;
  using T = Time;
  using I = ElectricCurrent;
  using Th = ThermodynamicTemperature;
  using N = AmountOfSubstance;
  using J = LuminousIntensity;
};

template<typename D1, typename D2>
using MultiplyDimensions = Dimension<std::ratio_add<typename D1::L, typename D2::L>,
  std::ratio_add<typename D1::M, typename D2::M>,
  std::ratio_add<typename D1::T, typename D2::T>,
  std::ratio_add<typename D1::I, typename D2::I>,
  std::ratio_add<typename D1::Th, typename D2::Th>,
  std::ratio_add<typename D1::N, typename D2::N>,
  std::ratio_add<typename D1::J, typename D2::J>>;

template<typename D1, typename D2>
using DivideDimensions = Dimension<std::ratio_subtract<typename D1::L, typename D2::L>,
  std::ratio_subtract<typename D1::M, typename D2::M>,
  std::ratio_subtract<typename D1::T, typename D2::T>,
  std::ratio_subtract<typename D1::I, typename D2::I>,
  std::ratio_subtract<typename D1::Th, typename D2::Th>,
  std::ratio_subtract<typename D1::N, typename D2::N>,
  std::ratio_subtract<typename D1::J, typename D2::J>>;

using Dimensionless = Dimension<>;
using Length = Dimension<std::ratio<1>>;
using Mass = Dimension<std::ratio<0>, std::ratio<1>>;
using Time = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using ElectricCurrent = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using ThermodynamicTemperature = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using AmountOfSubstance = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using LuminousIntensity = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;

using Area = MultiplyDimensions<Length, Length>;
using Volume = MultiplyDimensions<Length, Area>;
using Frequency = DivideDimensions<Dimensionless, Time>;
using Velocity = DivideDimensions<Length, Time>;
using Acceleration = DivideDimensions<Velocity, Time>;
using Force = MultiplyDimensions<Mass, Acceleration>;
using Pressure = DivideDimensions<Force, Area>;


template<typename Dimension, typename ValueType, typename ScalingFactor = std::ratio<1>> class Unit
{
  ValueType value_;

public:
  // Publicly expose template parameters for use in conversions and checks
  using D = Dimension;
  using VT = ValueType;
  using SF = ScalingFactor;

  explicit constexpr Unit(const ValueType &value) : value_(value) {}

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

  template<typename OtherUnit> constexpr auto operator*(const OtherUnit &other) const
  {
    static_assert(std::is_same<ValueType, typename OtherUnit::VT>::value, "Incompatible ValueTypes");
    using NewDimension = MultiplyDimensions<D, typename OtherUnit::D>;
    using NewScalingFactor = std::ratio_multiply<SF, typename OtherUnit::SF>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ value_ * other.getValue() };
  }

  template<typename OtherUnit> constexpr auto operator/(const OtherUnit &other) const
  {
    static_assert(std::is_same<ValueType, typename OtherUnit::VT>::value, "Incompatible ValueTypes");
    using NewDimension = DivideDimensions<D, typename OtherUnit::D>;
    using NewScalingFactor = std::ratio_divide<SF, typename OtherUnit::SF>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ value_ / other.getValue() };
  }
};

using Meter = Unit<Length, double>;// Base unit of Length
using Kilometer = Unit<Length, double, std::ratio<1000>>;

using Second = Unit<Time, double>;// Base unit of Time
using Millisecond = Unit<Time, double, std::ratio<1, 1000>>;
using Minute = Unit<Time, double, std::ratio<60>>;
using Hour = Unit<Time, double, std::ratio<3600>>;
using Hertz = Unit<Frequency, double>;