#pragma once

//////////////////////////////////////////////////////////////////////////
// STAC by Yooji.
//////////////////////////////////////////////////////////////////////////

// SITM
#include "_yooji_2014_SITM.h"

// VCL_3DO
#include "_VCL_2014_3DO.h"

// GPU.
#include "_yooji_2017_cuda_object_scanner.cuh"

// Object scanner.
#include "Util/_yooji_scanner_defines.h"

#include "Object/CPU/_yooji_scanner_parameter.h"
#include "Object/CPU/_yooji_camera_parameters.h"

#include "Object/CPU/_yooji_edge_map.h"
#include "Object/CPU/_yooji_image_pyramid.h"

//#include "Object/CPU/_yooji_camera_information.h" // for opencv.

#include "Object/CPU/_yooji_matrix_rgbd.h"
#include "Object/CPU/_yooji_cube_tsdf_float.h"
#include "Object/CPU/_yooji_tracking_state.h"

// Utilities.
#include "Util/_yooji_math_inlines.h"
#include "Util/_yooji_frame_capture.h"
#include "Util/_yooji_scanner_display.h"
#include "Util/_yooji_evaluation.h"

#include "Engine/CPU/_yooji_image_processor.h"
#include "Engine/CPU/_yooji_image_matcher.h"
// ================================================
// for Visual-hull shield.
#include "Engine/CPU/_yooji_object_extractor.h"
#include "Engine/CPU/_yooji_shape_from_silhouette.h"
// ================================================
#include "Engine/CPU/_yooji_camera_tracker.h"
#include "Engine/CPU/_yooji_volume_integrator.h"



#include "Engine/CPU/_yooji_object_scanner.h"