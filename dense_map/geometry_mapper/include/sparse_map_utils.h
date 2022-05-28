/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// This header file will be included when building without ROS and
// other Astrobee dependencies.  It will make some empty classes to
// make compilation succeed.

#ifndef SPARSE_MAP_UTILS_H_
#define SPARSE_MAP_UTILS_H_

#include <sparse_mapping/sparse_map.h>
#include <boost/shared_ptr.hpp>
#include <string>

namespace dense_map {

// Rebuild the map given updated intrinsics and camera poses
// TODO(oalexan1): This must be integrated in astrobee.
void RebuildMap(std::string const& map_file,  // Will be used for temporary saving of aux data
                camera::CameraParameters const& cam_params, double min_rays_angle, bool verbose,
                // Outputs
                boost::shared_ptr<sparse_mapping::SparseMap> sparse_map);
}  // end namespace dense_map

#endif  // SPARSE_MAP_UTILS_H_
