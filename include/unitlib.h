#include <concepts>
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
using AmountOfSubstance =
  Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using LuminousIntensity =
  Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;

using Area = MultiplyDimensions<Length, Length>;
using Volume = MultiplyDimensions<Length, Area>;
using Frequency = DivideDimensions<Dimensionless, Time>;
using Velocity = DivideDimensions<Length, Time>;
using Acceleration = DivideDimensions<Velocity, Time>;
using Force = MultiplyDimensions<Mass, Acceleration>;
using Pressure = DivideDimensions<Force, Area>;
using Energy = MultiplyDimensions<Force, Length>;
using Power = DivideDimensions<Energy, Time>;


template<typename Dimension, typename ValueType, typename ScalingFactor = std::ratio<1>>
  requires std::integral<ValueType> || std::floating_point<ValueType>
class Unit
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
  template<typename OtherUnit>
    requires std::is_same_v<Dimension, typename OtherUnit::D>
  constexpr operator OtherUnit() const
  {
    // Convert from this unit to the base unit, then from the base unit to the target unit
    return OtherUnit(value_ * (ScalingFactor::num / static_cast<ValueType>(ScalingFactor::den))
                     * (OtherUnit::SF::den / static_cast<ValueType>(OtherUnit::SF::num)));
  }

  template<typename OtherUnit>
    requires std::is_same_v<ValueType, typename OtherUnit::VT>
  constexpr auto operator*(const OtherUnit &other) const
  {
    using NewDimension = MultiplyDimensions<D, typename OtherUnit::D>;
    using NewScalingFactor = std::ratio_multiply<SF, typename OtherUnit::SF>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ value_ * other.getValue() };
  }

  template<typename OtherUnit>
    requires std::is_same_v<ValueType, typename OtherUnit::VT>
  constexpr auto operator/(const OtherUnit &other) const
  {
    using NewDimension = DivideDimensions<D, typename OtherUnit::D>;
    using NewScalingFactor = std::ratio_divide<SF, typename OtherUnit::SF>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ value_ / other.getValue() };
  }

  template<typename OtherUnit>
    requires std::is_same_v<Dimension, typename OtherUnit::D> && std::is_same_v<ValueType, typename OtherUnit::VT>
  constexpr auto operator+(const OtherUnit &other) const
  {
    ValueType otherValueInThisUnitScale =
      other.getValue() * (static_cast<ValueType>(OtherUnit::SF::num) / static_cast<ValueType>(OtherUnit::SF::den))
      / (static_cast<ValueType>(ScalingFactor::num) / static_cast<ValueType>(ScalingFactor::den));

    // Perform addition in the current unit's scale
    return Unit<Dimension, ValueType, ScalingFactor>(value_ + otherValueInThisUnitScale);
  }

  template<typename OtherUnit>
    requires std::is_same_v<Dimension, typename OtherUnit::D> && std::is_same_v<ValueType, typename OtherUnit::VT>
  constexpr auto operator-(const OtherUnit &other) const
  {
    ValueType otherValueInThisUnitScale =
      other.getValue() * (static_cast<ValueType>(OtherUnit::SF::num) / static_cast<ValueType>(OtherUnit::SF::den))
      / (static_cast<ValueType>(ScalingFactor::num) / static_cast<ValueType>(ScalingFactor::den));

    // Perform addition in the current unit's scale
    return Unit<Dimension, ValueType, ScalingFactor>(value_ - otherValueInThisUnitScale);
  }
};

// SI units
template<typename ValueType> using Meter = Unit<Length, ValueType>;

template<typename ValueType> using Kilogram = Unit<Mass, ValueType>;

template<typename ValueType> using Second = Unit<Time, ValueType>;

template<typename ValueType> using Ampere = Unit<ElectricCurrent, ValueType>;

template<typename ValueType> using Kelvin = Unit<ThermodynamicTemperature, ValueType>;

template<typename ValueType> using Mol = Unit<AmountOfSubstance, ValueType>;

template<typename ValueType> using Candela = Unit<LuminousIntensity, ValueType>;

template<typename ValueType> using Centimeter = Unit<Length, ValueType, std::ratio<1, 100>>;
template<typename ValueType> using Kilometer = Unit<Length, ValueType, std::ratio<1000>>;
template<typename ValueType> using Millisecond = Unit<Time, ValueType, std::ratio<1, 1000>>;
template<typename ValueType> using Minute = Unit<Time, ValueType, std::ratio<60>>;
template<typename ValueType> using Hour = Unit<Time, ValueType, std::ratio<3600>>;
template<typename ValueType> using Hertz = Unit<Frequency, ValueType>;