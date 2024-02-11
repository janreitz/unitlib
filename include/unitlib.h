#include <compare>
#include <concepts>
#include <ratio>
#include <type_traits>

namespace Unitlib {

template<typename Length = std::ratio<0>,
  typename Mass = std::ratio<0>,
  typename Time = std::ratio<0>,
  typename ElectricCurrent = std::ratio<0>,
  typename Temperature = std::ratio<0>,
  typename AmountOfSubstance = std::ratio<0>,
  typename LuminousIntensity = std::ratio<0>>
struct Dimension
{
  // Publicly expose template parameters for use in conversions and checks
  using L = Length;
  using M = Mass;
  using T = Time;
  using I = ElectricCurrent;
  using Th = Temperature;
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

template<typename Dimension, typename ValueType, typename ScalingFactor, typename Offset>
requires std::is_arithmetic_v<ValueType>
class Unit;

// Primary template, defaults to false
template<typename> struct is_instance_of_unit : std::false_type
{
};

// Specialized template for an instantiation of Unit
template<typename Dimension, typename ValueType, typename ScalingFactor, typename Offset>
struct is_instance_of_unit<Unit<Dimension, ValueType, ScalingFactor, Offset>> : std::true_type
{
};

// Helper function to remove const, volatile, and reference qualifiers
template<typename T>
constexpr bool is_instance_of_unit_v = is_instance_of_unit<std::remove_cv_t<std::remove_reference_t<T>>>::value;

template<typename Dimension,
  typename ValueType,
  typename ScalingFactor = std::ratio<1>,
  typename Offset = std::ratio<0>>
requires std::is_arithmetic_v<ValueType>
class Unit
{
  // TODO Can this be a concept?
  static_assert(ScalingFactor::num != 0, "ScalingFactor must not be zero");

  ValueType value_;

public:
  // Publicly expose template parameters for use in conversions and checks
  using _Dimension = Dimension;
  using _ValueType = ValueType;
  using _ScalingFactor = ScalingFactor;
  using _Offset = Offset;

  explicit constexpr Unit(const ValueType &value) noexcept : value_(value) {}

  constexpr ValueType get_value() const noexcept { return value_; }

  static constexpr ValueType get_scaling_factor() noexcept
  {
    return static_cast<ValueType>(ScalingFactor::num) / static_cast<ValueType>(ScalingFactor::den);
  }

  static constexpr ValueType get_offset() noexcept
  {
    return static_cast<ValueType>(Offset::num) / static_cast<ValueType>(Offset::den);
  }

  template<typename TargetUnit>
  requires is_instance_of_unit_v<TargetUnit> && std::is_same_v<Dimension,
    typename TargetUnit::_Dimension> && std::is_same_v<ValueType, typename TargetUnit::_ValueType>
  constexpr ValueType get_value_in() const noexcept
  {
    const ValueType valueInTargetUnitScale =
      get_base_value() / TargetUnit::get_scaling_factor() - TargetUnit::get_offset();
    return valueInTargetUnitScale;
  }

  constexpr ValueType get_base_value() const noexcept { return (get_value() + get_offset()) * get_scaling_factor(); }

  // Conversion function to another unit within the same dimension
  template<typename OtherUnit>
  requires is_instance_of_unit_v<OtherUnit> && std::is_same_v<Dimension, typename OtherUnit::_Dimension>
  constexpr operator OtherUnit() const noexcept { return OtherUnit(this->template get_value_in<OtherUnit>); }

  template<typename OtherUnit>
  requires is_instance_of_unit_v<OtherUnit> && std::is_same_v<Unit::_Dimension, typename OtherUnit::_Dimension>
  constexpr auto operator<=>(const OtherUnit &other) const noexcept
  {
    return get_base_value() <=> other.get_base_value();
  }

  template<typename OtherUnit>
  requires is_instance_of_unit_v<OtherUnit> && std::is_same_v<Dimension,
    typename OtherUnit::_Dimension> && std::is_same_v<ValueType, typename OtherUnit::_ValueType>
  constexpr bool operator==(const OtherUnit &other) const noexcept
  {
    return get_base_value() == other.get_base_value();
  }

  template<typename OtherUnit>
  requires is_instance_of_unit_v<OtherUnit> && std::is_same_v<ValueType, typename OtherUnit::_ValueType>
  constexpr auto operator*(const OtherUnit &other) const noexcept
  {
    using NewDimension = MultiplyDimensions<_Dimension, typename OtherUnit::_Dimension>;
    using NewScalingFactor = std::ratio_multiply<_ScalingFactor, typename OtherUnit::_ScalingFactor>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ get_value() * other.get_value() };
  }

  template<typename OtherValueType>
  requires std::is_same_v<ValueType, OtherValueType>// Relax to compatibility
  constexpr auto operator*(const OtherValueType &other) const noexcept
  {
    return Unit<Dimension, std::common_type_t<ValueType, OtherValueType>, ScalingFactor>{ get_value() * other };
  }

  template<typename OtherUnit>
  requires is_instance_of_unit_v<OtherUnit> && std::is_same_v<ValueType, typename OtherUnit::_ValueType>
  constexpr auto operator/(const OtherUnit &other) const
  {
    using NewDimension = DivideDimensions<_Dimension, typename OtherUnit::_Dimension>;
    using NewScalingFactor = std::ratio_divide<_ScalingFactor, typename OtherUnit::_ScalingFactor>;
    return Unit<NewDimension, ValueType, NewScalingFactor>{ get_value() / other.get_value() };
  }

  template<typename OtherValueType>
  requires std::is_same_v<ValueType, OtherValueType>// Relax to compatibility
  constexpr auto operator/(const OtherValueType &other) const
  {
    return Unit<Dimension, std::common_type_t<ValueType, OtherValueType>, ScalingFactor>{ get_value() / other };
  }

  template<typename OtherUnit>
  requires is_instance_of_unit_v<OtherUnit> && std::is_same_v<Dimension,
    typename OtherUnit::_Dimension> && std::is_same_v<ValueType, typename OtherUnit::_ValueType>
  constexpr auto operator+(const OtherUnit &other) const noexcept
  {
    return Unit{ get_value() + other.template get_value_in<Unit>() };
  }

  template<typename OtherUnit>
  requires is_instance_of_unit_v<OtherUnit> && std::is_same_v<Dimension,
    typename OtherUnit::_Dimension> && std::is_same_v<ValueType, typename OtherUnit::_ValueType>
  constexpr auto operator-(const OtherUnit &other) const noexcept
  {
    return Unit{ get_value() - other.template get_value_in<Unit>() };
  }
};

template<typename Dimension, typename ValueType, typename ScalingFactor, typename Scalar>
requires std::is_arithmetic_v<Scalar>
constexpr auto operator*(const Scalar &scalar, const Unit<Dimension, ValueType, ScalingFactor> &unit) noexcept
{
  return Unit<Dimension, std::common_type_t<ValueType, Scalar>, ScalingFactor>{ scalar * unit.get_value() };
}

template<typename Dimension, typename ValueType, typename ScalingFactor, typename Scalar>
requires std::is_arithmetic_v<Scalar>
constexpr auto operator/(const Scalar &scalar, const Unit<Dimension, ValueType, ScalingFactor> &unit)
{
  return Unit<Dimension, std::common_type_t<ValueType, Scalar>, ScalingFactor>{ scalar / unit.get_value() };
}

template<typename Unit, typename ExpectedDimension>
concept HasDimension = std::is_same_v<typename Unit::_Dimension, ExpectedDimension>;

// Dimensions of SI base units
using Dimensionless = Dimension<>;
using Length = Dimension<std::ratio<1>>;
using Mass = Dimension<std::ratio<0>, std::ratio<1>>;
using Time = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using ElectricCurrent = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using Temperature = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using AmountOfSubstance =
  Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using LuminousIntensity =
  Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;

// Derived Dimensions
using Area = MultiplyDimensions<Length, Length>;
using Volume = MultiplyDimensions<Length, Area>;
using Frequency = DivideDimensions<Dimensionless, Time>;
using Velocity = DivideDimensions<Length, Time>;
using Acceleration = DivideDimensions<Velocity, Time>;
using Force = MultiplyDimensions<Mass, Acceleration>;
using Pressure = DivideDimensions<Force, Area>;
using Energy = MultiplyDimensions<Force, Length>;
using Power = DivideDimensions<Energy, Time>;
using ElectricCharge = MultiplyDimensions<Time, ElectricCurrent>;
using ElectricPotential = DivideDimensions<Power, ElectricCurrent>;
using ElectricalResistance = DivideDimensions<ElectricPotential, ElectricCurrent>;
using ElectricalConductance = DivideDimensions<Dimensionless, ElectricalResistance>;
using Capacitance = DivideDimensions<ElectricCharge, ElectricPotential>;
using MagneticFlux = MultiplyDimensions<ElectricPotential, Time>;
using MagneticFluxDensity = DivideDimensions<MagneticFlux, Area>;
using Inductance = DivideDimensions<MagneticFlux, ElectricCurrent>;
using LuminousFlux = LuminousIntensity;
using Illuminance = DivideDimensions<LuminousIntensity, Area>;
using RadioactiveDose = DivideDimensions<Energy, Mass>;
using KatalyticActivity = DivideDimensions<AmountOfSubstance, Time>;

// Non SI dimensions
using KinematicViscosity = DivideDimensions<Area, Time>;

// SI base units
template<typename ValueType> using Meter = Unit<Length, ValueType>;
template<typename ValueType> using Kilogram = Unit<Mass, ValueType>;
template<typename ValueType> using Second = Unit<Time, ValueType>;
template<typename ValueType> using Ampere = Unit<ElectricCurrent, ValueType>;
template<typename ValueType> using Kelvin = Unit<Temperature, ValueType>;
template<typename ValueType> using Mole = Unit<AmountOfSubstance, ValueType>;
template<typename ValueType> using Candela = Unit<LuminousIntensity, ValueType>;

// SI derived units
template<typename ValueType> using Radian = Unit<Dimensionless, ValueType>;
template<typename ValueType> using Steradian = Unit<Dimensionless, ValueType>;
template<typename ValueType> using Hertz = Unit<Frequency, ValueType>;
template<typename ValueType> using Newton = Unit<Force, ValueType>;
template<typename ValueType> using Pascal = Unit<Pressure, ValueType>;
template<typename ValueType> using Joule = Unit<Energy, ValueType>;
template<typename ValueType> using Watt = Unit<Power, ValueType>;
template<typename ValueType> using Coulomb = Unit<ElectricCharge, ValueType>;
template<typename ValueType> using Volt = Unit<ElectricPotential, ValueType>;
template<typename ValueType> using Farad = Unit<Capacitance, ValueType>;
template<typename ValueType> using Ohm = Unit<ElectricalResistance, ValueType>;
template<typename ValueType> using Siemens = Unit<ElectricalConductance, ValueType>;
template<typename ValueType> using Weber = Unit<MagneticFlux, ValueType>;
template<typename ValueType> using Tesla = Unit<MagneticFluxDensity, ValueType>;
template<typename ValueType> using Henry = Unit<Inductance, ValueType>;
template<typename ValueType> using DegreeCelsius = Unit<Temperature, double, std::ratio<1>, std::ratio<27315, 100>>;
template<typename ValueType> using Lumen = Unit<LuminousFlux, ValueType>;
template<typename ValueType> using Lux = Unit<Illuminance, ValueType>;
template<typename ValueType> using Becquerel = Unit<RadioactiveDose, ValueType>;
template<typename ValueType> using Gray = Unit<RadioactiveDose, ValueType>;
template<typename ValueType> using Sievert = Unit<RadioactiveDose, ValueType>;
template<typename ValueType> using Katal = Unit<KatalyticActivity, ValueType>;

// Scaled units
template<typename ValueType> using Centimeter = Unit<Length, ValueType, std::ratio<1, 100>>;
template<typename ValueType> using Kilometer = Unit<Length, ValueType, std::ratio<1000>>;
template<typename ValueType> using Millisecond = Unit<Time, ValueType, std::ratio<1, 1000>>;
template<typename ValueType> using Minute = Unit<Time, ValueType, std::ratio<60>>;
template<typename ValueType> using Hour = Unit<Time, ValueType, std::ratio<3600>>;
template<typename ValueType> using Hertz = Unit<Frequency, ValueType>;

namespace Literals {
  inline constexpr Meter<double> operator"" _m(long double val) noexcept
  {
    return Meter<double>(static_cast<double>(val));
  }
  inline constexpr Meter<int64_t> operator"" _m(unsigned long long val) noexcept
  {
    return Meter<int64_t>(static_cast<int64_t>(val));
  }
  inline constexpr Kilogram<double> operator"" _kg(long double val) noexcept
  {
    return Kilogram<double>(static_cast<double>(val));
  }
  inline constexpr Kilogram<int64_t> operator"" _kg(unsigned long long val) noexcept
  {
    return Kilogram<int64_t>(static_cast<int64_t>(val));
  }
  inline constexpr Second<double> operator"" _s(long double val) noexcept
  {
    return Second<double>(static_cast<double>(val));
  }
  inline constexpr Second<int64_t> operator"" _s(unsigned long long val) noexcept
  {
    return Second<int64_t>(static_cast<int64_t>(val));
  }
  inline constexpr Ampere<double> operator"" _A(long double val) noexcept
  {
    return Ampere<double>(static_cast<double>(val));
  }
  inline constexpr Ampere<int64_t> operator"" _A(unsigned long long val) noexcept
  {
    return Ampere<int64_t>(static_cast<int64_t>(val));
  }
  inline constexpr Kelvin<double> operator"" _K(long double val) noexcept
  {
    return Kelvin<double>(static_cast<double>(val));
  }
  inline constexpr Kelvin<int64_t> operator"" _K(unsigned long long val) noexcept
  {
    return Kelvin<int64_t>(static_cast<int64_t>(val));
  }
  inline constexpr Mole<double> operator"" _mol(long double val) noexcept
  {
    return Mole<double>(static_cast<double>(val));
  }
  inline constexpr Mole<int64_t> operator"" _mol(unsigned long long val) noexcept
  {
    return Mole<int64_t>(static_cast<int64_t>(val));
  }
  inline constexpr Candela<double> operator"" _cd(long double val) noexcept
  {
    return Candela<double>(static_cast<double>(val));
  }
  inline constexpr Candela<int64_t> operator"" _cd(unsigned long long val) noexcept
  {
    return Candela<int64_t>(static_cast<int64_t>(val));
  }
}// namespace Literals

template<typename Unit, typename ExpectedDimension, typename ExpectedValueType> constexpr void unit_check() noexcept
{
  static_assert(std::is_same_v<typename Unit::_Dimension, ExpectedDimension>, "Dimension mismatch");
  static_assert(std::is_same_v<typename Unit::_ValueType, ExpectedValueType>, "ValueType mismatch");
}
}// namespace Unitlib