/*
 * navigation.h
 *
 *  Created on: Aug 24, 2026
 *      Author: lethanhtra
 */

#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

#include "main.h"
float calculate_distance(double lat1, double lon1, double lat2, double lon2);
float calculate_bearing(double lat1, double lon1, double lat2, double lon2);

#endif /* INC_NAVIGATION_H_ */
