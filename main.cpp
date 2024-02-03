#include "include/unitlib.h"

#include <iostream>

int main(int, char **)
{
  constexpr Unit::Meter<double> distance(100);
  constexpr Unit::Kilometer<double> distance_km = distance;
  constexpr Unit::Centimeter<double> addition_result = distance + distance_km;
  constexpr Unit::Meter<double> subtraction_result = distance - distance_km;

  std::cout << "Centimeter: " << double(Unit::Centimeter<double>(distance) * 5) << '\n';
  std::cout << "Kilometer: " << double(Unit::Kilometer<double>(distance)) << '\n';
  std::cout << "Addition result: " << double(addition_result) << '\n';
  std::cout << "Subtraction result: " << double(subtraction_result) << '\n';

  constexpr Unit::Second<double> time(30);
  constexpr Unit::Millisecond<double> time_ms = time;
  constexpr Unit::Hour<double> time_h = time;
  std::cout << double(time_ms) << '\n';

  auto area = distance * distance;
  std::cout << "Square meters" << double(area) << '\n';

  constexpr Unit::Unit<Unit::Velocity, double> meters_per_second = distance_km / time_h;

  auto kilometers_per_hour = distance_km / time_h;

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(meters_per_second) << "[m/s]"
            << '\n';

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(kilometers_per_hour)
            << "[km/h]" << '\n';


  return 0;
}
