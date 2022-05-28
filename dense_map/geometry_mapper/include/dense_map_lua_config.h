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

#ifndef DENSE_MAP_LUA_CONFIG_H_
#define DENSE_MAP_LUA_CONFIG_H_

#include <config_reader/config_reader.h>
#include <camera/camera_params.h>
#include <dense_map_utils.h>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace dense_map {

// Read the transform from depth to given camera
void readCameraTransform(config_reader::ConfigReader& config, std::string const transform_str,
                         Eigen::Affine3d& transform);

// Read some transforms from the robot calibration file
void readLuaConfig                                                     // NOLINT
(// Inputs                                                              // NOLINT
 std::vector<std::string> const& cam_names,                             // NOLINT
 std::string const& nav_cam_to_body_trans_str,                          // NOLINT
 std::string const& haz_cam_depth_to_image_trans_str,                   // NOLINT
 // Outputs                                                             // NOLINT
 std::vector<camera::CameraParameters> & cam_params,                    // NOLINT
 std::vector<Eigen::Affine3d>          & nav_to_cam_trans,              // NOLINT
 std::vector<double>                   & nav_to_cam_timestamp_offset,   // NOLINT
 Eigen::Affine3d                       & nav_cam_to_body_trans,         // NOLINT
 Eigen::Affine3d                       & haz_cam_depth_to_image_trans); // NOLINT

// This is a wrapper hiding some things
void readLuaConfig(bool & have_rig_transforms, int & ref_cam_type,
                    std::vector<std::string> & cam_names,
                    std::vector<camera::CameraParameters> & cam_params,
                    std::vector<Eigen::Affine3d> & ref_to_cam_trans,
                    std::vector<Eigen::Affine3d> & depth_to_image,
                    std::vector<double> & ref_to_cam_timestamp_offsets);

// Save some transforms from the robot calibration file. This has some very fragile
// logic and cannot handle comments in the config file.
void writeLuaConfig                                                           // NOLINT
(std::vector<std::string>              const& cam_names,                        // NOLINT
 std::string                           const& haz_cam_depth_to_image_trans_str, // NOLINT
 std::vector<camera::CameraParameters> const& cam_params,                       // NOLINT
 std::vector<Eigen::Affine3d>          const& nav_to_cam_trans,                 // NOLINT
 std::vector<double>                   const& nav_to_cam_timestamp_offset,      // NOLINT
 Eigen::Affine3d                       const& haz_cam_depth_to_image_trans);    // NOLINT


// This is a wrapper for writeLuaConfig() hiding some things
void writeLuaConfig                                                        // NOLINT
(std::vector<std::string>              const& cam_names,                    // NOLINT
 std::vector<camera::CameraParameters> const& cam_params,                   // NOLINT
 std::vector<Eigen::Affine3d>          const& ref_to_cam_trans,             // NOLINT
 std::vector<double>                   const& ref_to_cam_timestamp_offset,  // NOLINT
 std::vector<Eigen::Affine3d>          const& depth_to_image_trans);        // NOLINT

}  // namespace dense_map

#endif  // DENSE_MAP_LUA_CONFIG_H_
