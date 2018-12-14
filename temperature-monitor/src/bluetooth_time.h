#include "arduino.h"

struct date_time_t{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
};
 
struct day_of_week_t {
  uint8_t day;
};

struct day_date_time_t {
  struct date_time_t date_time;
  struct day_of_week_t day_of_week;
};
 
struct exact_time_256_t {
  struct day_date_time_t day_date_time;
  uint8_t fractions_256;
};

struct current_time_t {
  struct exact_time_256_t exact_time_256;
  uint8_t adjust_reason;
};