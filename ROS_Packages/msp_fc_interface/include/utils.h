#pragma once

// #include <math.h>
// #include <stdint.h>

#define R2D (180.0 / 3.142)
#define D2R (3.142 / 180.0)
#define PI  (3.142)

// bound a value to a range [min,max]
inline float bound_f(float val, float min, float max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}

// wrap yaw angles between -180 and +180
inline float wrap_ang(float ang) {
	if (ang < -PI) {
		ang += 2 * PI;
	}
	if (ang > PI) {
		ang -= 2 * PI;
	}
	return ang;
}


//fonts color
#define COLOR_FBLACK      "\033[30;"
#define COLOR_FRED        "\033[31;"
#define COLOR_FGREEN      "\033[32;"
#define COLOR_FYELLOW     "\033[33;"
#define COLOR_FBLUE       "\033[34;"
#define COLOR_FPURPLE     "\033[35;"
#define COLOR_D_FGREEN    "\033[6;"
#define COLOR_FWHITE      "\033[7;"
#define COLOR_FCYAN       "\x1b[36m"

//background color
#define COLOR_BBLACK      "40m"
#define COLOR_BRED        "41m"
#define COLOR_BGREEN      "42m"
#define COLOR_BYELLOW     "43m"
#define COLOR_BBLUE       "44m"
#define COLOR_BPURPLE     "45m"
#define COLOR_D_BGREEN    "46m"
#define COLOR_BWHITE      "47m"

//end color
#define COLOR_NONE        "\033[0m"