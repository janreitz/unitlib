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
To start using Unitlib in your project, include the provided header and make sure your compiler supports C++20, as Unitlib makes extensive use of C++20 features such as `concepts` and `constexpr`.

```cpp
#include "unitlib.hpp"
```
You can also install Unitlib using CMake

```sh
mkdir build && cd build
cmake ..
make install
```
And then use `find_package` in your own `CMakeLists.txt` to setup include paths and C++ version.
```py
find_package(unitlib REQUIRED)
```

## Usage

Use predefined unit types for SI units by stating their types or using literals:

```cpp
using namespace Unitlib;

Meter<double> length(2.0);
Second<double> time(5.0);

using namespace Unitlib::Literals;
auto temperature = 1_K;
auto substance_amount = 1.0_mol;
```
### Defining new units
You can define new Units with existing dimensions by stating `ValueType`, as well as `ScalingFactor` and `Offset` relative to the base SI units:
```cpp
// Only scaling factor, offset defaults to `std::ratio<0>`
using Inch = Unit<Length, double, std::ratio<254, 10000>>;
static_assert(Inch(1.0).get_value_in<Meter<double>>() == 0.0254);

// Only offset with neutral scaling factor `std::ratio<1>`
using Celsius = Unit<Temperature, double, std::ratio<1>, std::ratio<27315, 100>>;
static_assert(Celsius(23.0).get_base_value() == 296.15);

// Both offset and scaling factor
using Fahrenheit = Unit<Temperature, double, std::ratio<5, 9>, std::ratio<45967, 100>>;
static_assert(std::abs(Celsius(23.0).get_value_in<Fahrenheit>() - 73.4) < 0.1);
static_assert(std::abs(Fahrenheit(73.4).get_base_value() - 296.15) < 0.1);
```

[`std::ratio'](https://en.cppreference.com/w/cpp/numeric/ratio/ratio) provides convenient typedefs for SI prefixes:
```cpp
#include <ratio>
using Kilohertz = Unit<Frequency, double, std::kilo>;
```

### Defining new dimensions
Define new dimensions by explicitly stating them in terms of SI base unit dimensions or by deriving them from existing dimensions:
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
auto flow_speed = 1_m/1_s;
auto characteristic_length = 1_m;
auto kinematic_viscosity = 1_m * 1_m/1_s;
auto reynolds_number = flow_speed * characteristic_length/kinematic_viscosity;
unit_check<decltype(reynolds_number), Dimensionless, double>();
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

Please install the pre-commit hook before committing any potential contributions, by performing this command in the repository root:
```bash
ln -s -f ../../hooks/pre-commit .git/hooks/pre-commit
```
This ensures that tests pass and the code is formatted consistently on each commit.
