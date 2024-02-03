#include "include/unitlib.h"

#include <iostream>

int main(int, char **)
{
  constexpr Unitlib::Meter<double> distance(100);
  constexpr Unitlib::Kilometer<double> distance_km = distance;
  constexpr Unitlib::Centimeter<double> addition_result = distance + distance_km;
  constexpr Unitlib::Meter<double> subtraction_result = distance - distance_km;

  std::cout << "Centimeter: " << double(Unitlib::Centimeter<double>(distance) * 5) << '\n';
  std::cout << "Kilometer: " << double(Unitlib::Kilometer<double>(distance)) << '\n';
  std::cout << "Addition result: " << double(addition_result) << '\n';
  std::cout << "Subtraction result: " << double(subtraction_result) << '\n';

  constexpr Unitlib::Second<double> time(30);
  constexpr Unitlib::Millisecond<double> time_ms = time;
  constexpr Unitlib::Hour<double> time_h = time;
  std::cout << double(time_ms) << '\n';

  auto area = distance * distance;
  std::cout << "Square meters" << double(area) << '\n';

  constexpr Unitlib::Unit<Unitlib::Velocity, double> meters_per_second = distance_km / time_h;

  auto kilometers_per_hour = distance_km / time_h;

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(meters_per_second) << "[m/s]"
            << '\n';

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(kilometers_per_hour)
            << "[km/h]" << '\n';


  return 0;
}
