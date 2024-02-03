#include "include/unitlib.h"

#include <iostream>

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
