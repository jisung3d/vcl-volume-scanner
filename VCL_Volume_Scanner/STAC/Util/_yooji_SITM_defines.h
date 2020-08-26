//////////////////////////////////////////////////////////////////////
// _yooji_SITM_defines.h
//////////////////////////////////////////////////////////////////////

#pragma once

// defines
// for SLIC.
#define KV_SLIC_SUPERPIXEL_SIZE 200
#define KV_SLIC_COMPACTNESS 10.0
#define KV_SLIC_MAXIMUM_NEIGHBOR_NUM_PLUS_1 16

// for GMM color segmentation.
#define NUMBER_OF_CLUSTERS 1

#define KV_MAX_ITERATION_FOR_INITIAL_MEAN 2000
#define KV_COLOR_MODEL_RGB 0			// 3-dimensional feature.
#define KV_COLOR_MODEL_HUE 1			// 1-dimensional feature.
#define KV_COLOR_MODEL_PCS 2			// 3-dimensional feature.

#define KV_MIN_MEAN_DIFFERENCE_RGB 100.0f
#define KV_MIN_MEAN_DIFFERENCE_HUE 40.0f
#define KV_MIN_MEAN_DIFFERENCE_PCS 10.0f

#define KV_MAX_MEAN_SHIFT_RGB 2.0f
#define KV_MAX_MEAN_SHIFT_HUE 2.0f
#define KV_MAX_MEAN_SHIFT_PCS 1.0f

// macros
#define SQUARE(x) (x)*(x)
#define ROUNDF(x) (x<0.0f) ? (int)(x-0.5f) : (int)(x+0.5f)
#define DIST_CIRCULAR(x) ( (x<=180.0f) ? x : 360.0f-x );
