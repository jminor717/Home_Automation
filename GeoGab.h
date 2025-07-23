#include "esphome.h"
using namespace esphome;

class GeoGab {
public:
    struct settings_t {
        string name = "Main";
        int timeout = 60;
    } settings;

    struct timeout_t {
        int lowwater = 30;
        int waterlock = 60;
    } timeout;

    struct color_t {
        Color bgreen = Color(84, 130, 53);
        Color bgrey = Color(100, 100, 100);
        Color bdgrey = Color(50, 50, 50);
        Color bblue = Color(77, 98, 253);
        Color bdblue = Color(90, 90, 120);
    } color;
} geogab;