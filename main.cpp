#include "include/unitlib.h"

#include <iostream>

int main(int, char **)
{
  constexpr Meter<double> distance(100);
  constexpr Kilometer<double> distance_km = distance;

  std::cout << "Kilometers: " << double(distance_km) << '\n';

  constexpr Second<double> time(30);
  constexpr Millisecond<double> time_ms = time;
  constexpr Hour<double> time_h = time;
  std::cout << double(time_ms) << '\n';

  auto area = distance * distance;
  std::cout << "Square meters" << double(area) << '\n';

  constexpr Unit<Velocity, double> meters_per_second = distance_km / time_h;

  auto kilometers_per_hour = distance_km / time_h;

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(meters_per_second) << "[m/s]"
            << '\n';

  std::cout << double(distance_km) << " [km] / " << double(time_h) << " [h] = " << double(kilometers_per_hour)
            << "[km/h]" << '\n';


  return 0;
}
