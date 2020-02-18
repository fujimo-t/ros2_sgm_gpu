/***********************************************************************
  Copyright (C) 2019 Hironori Fujimoto

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
#ifndef DISPARITY_METHOD_H_
#define DISPARITY_METHOD_H_

#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "util.h"
#include "configuration.h"
#include "costs.h"
#include "hamming_cost.h"
#include "median_filter.h"
#include "cost_aggregation.h"
#include "debug.h"
#include "left_right_consistency.h"

void init_disparity_method(const uint8_t _p1, const uint8_t _p2);
void compute_disparity_method(cv::Mat left, cv::Mat right, cv::Mat* disparity, float *elapsed_time_ms, bool check_consistency);
void finish_disparity_method();
static void free_memory();

#endif /* DISPARITY_METHOD_H_ */
