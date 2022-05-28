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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sparse_map_utils.h>
#include <sparse_mapping/reprojection.h>
#include <sparse_mapping/tensor.h>

namespace dense_map {

// Rebuild the map given updated intrinsics and camera poses
// TODO(oalexan1): This must be integrated in astrobee.
void RebuildMap(std::string const& map_file,  // Will be used for temporary saving of aux data
                camera::CameraParameters const& cam_params,
                double min_rays_angle, bool verbose,
                // Outputs
                boost::shared_ptr<sparse_mapping::SparseMap> sparse_map) {
  std::string rebuild_detector = "SURF";
  std::cout << "Rebuilding map with " << rebuild_detector << " detector.";

  // Set programatically the command line option for astrobee's map
  // building min angle based on the corresponding refiner flag.
  std::ostringstream oss;
  oss << min_rays_angle;
  std::string min_valid_angle = oss.str();
  google::SetCommandLineOption("min_valid_angle", min_valid_angle.c_str());
  if (!gflags::GetCommandLineOption("min_valid_angle", &min_valid_angle))
    LOG(FATAL) << "Failed to get the value of --min_valid_angle in Astrobee "
               << "map-building software.\n";
  // The newline below is due to the sparse map software not putting a newline
  std::cout << "\nSetting --min_valid_angle " << min_valid_angle << ".\n";

  // Make the map rebuilding in the Astrobee software not print so much matching info
  if (!verbose)
    google::SetCommandLineOption("silent_matching", "true");

  // Copy some data to make sure it does not get lost on resetting things below
  std::vector<Eigen::Affine3d>    world_to_ref_t = sparse_map->cid_to_cam_t_global_;
  std::vector<std::map<int, int>> pid_to_cid_fid = sparse_map->pid_to_cid_fid_;

  // Ensure the new camera parameters are set
  sparse_map->SetCameraParameters(cam_params);

  std::cout << "Detecting features.";
  sparse_map->DetectFeatures();

  std::cout << "Matching features.";
  // Borrow from the original map which images should be matched with which.
  sparse_map->cid_to_cid_.clear();
  for (size_t p = 0; p < pid_to_cid_fid.size(); p++) {
    std::map<int, int> const& track = pid_to_cid_fid[p];  // alias
    for (std::map<int, int>::const_iterator it1 = track.begin();
         it1 != track.end() ; it1++) {
      for (std::map<int, int>::const_iterator it2 = it1;
           it2 != track.end() ; it2++) {
        if (it1->first != it2->first) {
          // Never match an image with itself
          sparse_map->cid_to_cid_[it1->first].insert(it2->first);
        }
      }
    }
  }

  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(map_file),
                                sparse_mapping::MatchesFile(map_file), sparse_map.get());
  for (size_t i = 0; i < world_to_ref_t.size(); i++)
    sparse_map->SetFrameGlobalTransform(i, world_to_ref_t[i]);

  // Wipe file that is no longer needed
  try {
    std::remove(sparse_mapping::EssentialFile(map_file).c_str());
  }catch(...) {}

  std::cout << "Building tracks.";
  bool rm_invalid_xyz = true;  // by now cameras are good, so filter out bad stuff
  sparse_mapping::BuildTracks(rm_invalid_xyz,
                              sparse_mapping::MatchesFile(map_file),
                              sparse_map.get());

  // It is essential that during re-building we do not vary the
  // cameras. Those were usually computed with a lot of SURF features,
  // while rebuilding is usually done with many fewer ORGBRISK
  // features.
  bool fix_cameras = true;
  if (fix_cameras)
    std::cout << "Performing bundle adjustment with fixed cameras.";
  else
    std::cout << "Performing bundle adjustment while floating cameras.";

  sparse_mapping::BundleAdjust(fix_cameras, sparse_map.get());
}

}  // end namespace dense_map
