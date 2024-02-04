# Unitlib: A Compile-Time Unit Handling Library in C++

Unitlib is a header-only C++ library designed to facilitate explicit handling of physical units in C++ programs to enforce unit correctness at compile time. 
It aims to provide a robust foundation for working with units in a type-safe manner, ensuring that unit-related errors can be caught early in the development process. 
The design philosophy of Unitlib is to offer clear and intuitive unit handling without getting in the way and maintaining minimal runtime overhead.

## Features
- **Type Safety**: Enforces correct unit operations and conversions at compile time, reducing runtime errors and bugs related to unit handling.
- **Dimensional Analysis**: Supports operations between units, ensuring that the resulting units are dimensionally consistent (e.g., multiplying length by length yields an area).
- **Unit Conversions**: Provides facilities for converting between units, including handling of scaling factors (e.g., meters to kilometers).
- **Ergonomics**: Introduces user-defined literals for intuitive unit specification (e.g., `1.0_m` for meters, `1.0_s` for seconds).
- **Extensibility**: Easily extendable to include new units and dimensions as needed.

## Getting Started
To start using Unitlib in your project, include the provided header file in your C++ source files:

```cpp
#include "Unitlib.h"
```
Make sure your compiler supports C++20, as Unitlib makes extensive use of C++20 features such as `concepts` and `constexpr`.

## Usage

### Defining Units
Use predefined unit types for SI units by stating their types or using literals:

```cpp
using namespace Unitlib;

Meter<double> length(2.0);
Second<double> time(5.0);

using namespace Unitlib::Literals;
auto temperature = 1_K;
auto substanceAmount = 1.0_mol;
```

You can create new Units with existing dimensions:
```cpp
using Inch = Unit<Length, double, std::ratio<254, 10000>>;
static_assert(Inch(1.0).getValueIn<Meter<double>>() == 0.0254);
```

Create new dimensions by explicitly stating them in terms of SI base unit dimensions or by deriving them from existing dimensions:
```cpp
using Velocity = Dimension<
    std::ratio<1>,  // Length
    std::ratio<0>,  // Mass
    std::ratio<-1>, // Time
    std::ratio<0>,  // Electric current
    std::ratio<0>,  // Temperature
    std::ratio<0>,  // Amount of substance
    std::ratio<0>>; // Luminous intensity

using Torque = MultiplyDimensions<Force, Length>;
using NewtonMeters = Unit<Torque, double>;
```

### Performing Operations
Unitlib allows for natural arithmetic operations while ensuring type safety, even when converting between units of different dimensions:

```cpp
auto area = length * length;        // Result is in square meters
auto velocity = length / time;      // Result is in meters per second
```

You can specifically check the resulting units:

```cpp
// Check dimension and value type at compile time
unit_check<decltype(area), Area, double>(); 
```

Or always use literals and auto and worry about units only when defining inputs and querying the result:
```cpp
auto flowSpeed = 1_m/1_s;
auto characteristicLength = 1_m;
auto kinematicViscosity = 1_m * 1_m/1_s;
auto reynoldsNumber = flowSpeed * characteristicLength/kinematicViscosity;
unit_check<decltype(reynoldsNumber), Dimensionless, double>();
```

### Constrain template parameters

There is also a concept to constrain template parameters to be of specific `Dimension`:

```cpp
template<typename U, typename L, typename v>
requires HasDimension<U, Velocity> && HasDimension<L, Length> && HasDimension<v, KinematicViscosity>
constexpr auto reynolds_number(const U& flow_speed, const L& characteristic_length, const v& kinematic_viscosity) {
    return flow_speed * characteristic_length / kinematic_viscosity;
}
```

## Contributing
Contributions, suggestions, and feedback are highly welcome. Whether it's adding new units, improving the library's interface, or optimizing existing code, your input can help make Unitlib better.