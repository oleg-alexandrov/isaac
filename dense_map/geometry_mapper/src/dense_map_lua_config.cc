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

#include <glog/logging.h>

#include <system_utils.h>
#include <dense_map_utils.h>
#include <dense_map_lua_config.h>
#include <msg_conversions/msg_conversions.h>

#include <iostream>
#include <fstream>

namespace dense_map {
// Some private low-level functions

std::string number_to_string(double val) {
  std::ostringstream oss;
  oss.precision(8);
  oss << val;
  return oss.str();
}

std::string vector_to_string(Eigen::VectorXd v) {
  std::ostringstream oss;
  oss.precision(8);
  oss << "{";
  for (int it = 0; it + 1 < v.size(); it++) oss << v[it] << ", ";
  if (v.size() > 0) oss << v[v.size() - 1] << "}";
  return oss.str();
}

std::string matrix_to_string(Eigen::MatrixXd const& M) {
  std::ostringstream oss;
  oss.precision(8);
  oss << "{\n";
  for (int row = 0; row < M.rows(); row++) {
    oss << "    ";
    for (int col = 0; col < M.cols(); col++) {
      oss << M(row, col);
      if (row < M.rows() - 1 || col < M.cols() - 1) {
        oss << ",";
        if (col < M.cols() - 1) oss << " ";
      }
    }
    if (row < M.rows() - 1) oss << "\n";
  }

  oss << "}";
  return oss.str();
}

std::string affine_to_string(Eigen::Affine3d const& T) {
  std::ostringstream oss;
  oss.precision(8);
  Eigen::Quaterniond q = Eigen::Quaterniond(T.linear());
  Eigen::Vector3d t = T.translation();
  oss << "(vec3(" << t[0] << ", " << t[1] << ", " << t[2] << "), quat4(" << q.x() << ", " << q.y() << ", " << q.z()
      << ", " << q.w() << "))";
  return oss.str();
}

std::string intrinsics_to_string(Eigen::Vector2d const& f, Eigen::Vector2d const& o) {
  std::ostringstream oss;
  oss.precision(8);
  oss << "{\n"
      << "      " << f[0] << ", 0.0, " << o[0] << ",\n"
      << "      0.0, " << f[1] << ", " << o[1] << ",\n"
      << "      0.0, 0.0, 1.0\n"
      << "    }";
  return oss.str();
}

// Find in the given text the given value, followed by some spaces perhaps, and followed
// by the equal sign. Start at position beg. Return the position after the equal sign.
int robust_find(std::string const& text, std::string const& val, int beg) {
  // Need a while loop since there may be more than one candidate. Stop at the first.
  int text_len = text.size();
  while (beg < text_len) {
    beg = text.find(val, beg);
    if (beg == std::string::npos) return beg;

    // TODO(oalexan1): Must skip comments.  From this position must
    // read back towards the beginning of the current line and see if
    // the text "--" is encountered, which would mean that this
    // position is on a comment and hence the search must continue.

    beg += val.size();  // advance

    // Look for spaces and the equal sign
    bool found = true;
    for (int it = beg; it < text_len; it++) {
      if (text[it] == '=') {
        beg = it + 1;
        return beg;
      }

      // No equal sign yet. So must have spaces until then.
      if (text[it] != ' ' && text[it] != '\t' && text[it] != '\n' && text[it] != '\r') {
        found = false;
        beg = it;
        break;
      }
    }

    // No luck, will try again from the current position.
    if (!found) continue;
  }

  return beg;
}

// A fragile function to determine if the value of the given parameter has a brace
bool param_val_has_braces(std::string const& param_name, std::string const& parent,
                          // The text to search
                          std::string const& text) {
  int beg = 0;

  // First find the parent, if provided
  if (parent != "") {
    beg = robust_find(text, parent, beg);
    if (beg == std::string::npos) LOG(FATAL) << "Could not find the field '"
                                             << parent << " =' in the config file.";
  }

  // Find the param after the parent
  beg = robust_find(text, param_name, beg);
  if (beg == std::string::npos) {
    std::string msg;
    if (parent != "") msg = " Tried to locate it after field '" + parent + "'.";
    LOG(FATAL) << "Could not find the field '" << param_name << " =' in the config file." << msg;
  }

  // Now we are positioned after the equal sign
  bool has_brace = false;
  while (beg < static_cast<int>(text.size())) {
    if (text[beg] == ' ') {
      beg++;
      continue;
    }

    if (text[beg] == '{') has_brace = true;

    break;
  }

  return has_brace;
}

// Replace a given parameter's value in the text. This is very fragile
// code, particularly it does not understand that text starting with
// "--" is a comment.
void replace_param_val(std::string const& param_name, std::string const& param_val,
                       std::string const& parent,
                       std::string const& beg_brace, std::string const& end_brace,
                       // The text to modify
                       std::string& text) {
  int beg = 0;

  // First find the parent, if provided
  if (parent != "") {
    beg = robust_find(text, parent, beg);
    if (beg == std::string::npos) LOG(FATAL) << "Could not find the field '"
                                             << parent << " =' in the config file.";
  }

  // Find the param after the parent
  beg = robust_find(text, param_name, beg);
  if (beg == std::string::npos) {
    std::string msg;
    if (parent != "") msg = " Tried to locate it after field '" + parent + "'.";
    LOG(FATAL) << "Could not find the field '" << param_name << " =' in the config file." << msg;
  }

  int end = beg + 1;
  if (beg_brace != "" && end_brace != "") {  // The text to replace is in braces
    if (beg_brace.size() != 1 && end_brace.size() != 1)
      LOG(FATAL) << "Expecting one character for each of " << beg_brace << " and " << end_brace;

    beg = text.find(beg_brace, beg);
    if (beg == std::string::npos) {
      LOG(FATAL) << "Failed to replace value for " << parent << " " << param_name
                 << " in the config file.";
    }
    end = beg + 1;

    // Find the matching brace
    int count = 1;
    while (end < static_cast<int>(text.size())) {
      if (text[end] == beg_brace[0])
        count++;
      else if (text[end] == end_brace[0])
        count--;
      if (count == 0) break;

      end++;
    }

  } else {
    // No braces, then just look for the next comma or newline
    end = std::min(text.find(",", beg), text.find("\n", beg));
    if (beg == std::string::npos) LOG(FATAL) << "Could not parse correctly " << param_name;
    end--;  // go before the found character
  }

  text = text.replace(beg, end - beg + 1, param_val);
}

// Read the transform from depth to given camera
void readCameraTransform(config_reader::ConfigReader& config, std::string const transform_str,
                         Eigen::Affine3d& transform) {
  Eigen::Vector3d T;
  Eigen::Quaterniond R;
  if (!msg_conversions::config_read_transform(&config, transform_str.c_str(), &T, &R))
    LOG(FATAL) << "Unspecified transform: " << transform_str << " for robot: "
               << getenv("ASTROBEE_ROBOT") << "\n";

  R.normalize();

  transform = Eigen::Affine3d(Eigen::Translation3d(T.x(), T.y(), T.z())) * Eigen::Affine3d(R);
}

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
 Eigen::Affine3d                       & haz_cam_depth_to_image_trans){ // NOLINT
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("transforms.config");
  if (!config.ReadFiles()) LOG(FATAL) << "Failed to read config files.";

  cam_params.clear();
  nav_to_cam_trans.clear();
  nav_to_cam_timestamp_offset.clear();
  for (size_t it = 0; it < cam_names.size(); it++) {
    camera::CameraParameters params = camera::CameraParameters(&config, cam_names[it].c_str());
    cam_params.push_back(params);

    std::string trans_str = "nav_cam_to_" + cam_names[it] + "_transform";
    if (cam_names[it] == "nav_cam") {
      // transforms from nav cam to itself
      nav_to_cam_trans.push_back(Eigen::Affine3d::Identity());
      nav_to_cam_timestamp_offset.push_back(0.0);
    } else {
      Eigen::Affine3d trans;
      readCameraTransform(config, trans_str, trans);
      nav_to_cam_trans.push_back(trans);

      std::string offset_str = "nav_cam_to_" + cam_names[it] + "_timestamp_offset";
      double offset;
      if (!config.GetReal(offset_str.c_str(), &offset))
        LOG(FATAL) << "Could not read value of " << offset_str
                   << " for robot: " << getenv("ASTROBEE_ROBOT");
      nav_to_cam_timestamp_offset.push_back(offset);
    }
  }

  // Read the remaining data
  readCameraTransform(config, nav_cam_to_body_trans_str, nav_cam_to_body_trans);

  Eigen::MatrixXd M(4, 4);
  config_reader::ConfigReader::Table mat(&config, haz_cam_depth_to_image_trans_str.c_str());
  int count = 0;
  for (int row = 0; row < M.rows(); row++) {
    for (int col = 0; col < M.cols(); col++) {
      count++;  // note that the count stats from 1
      if (!mat.GetReal(count, &M(row, col))) {
        LOG(FATAL) << "Could not read value of " << haz_cam_depth_to_image_trans_str
                   << " for robot: " << getenv("ASTROBEE_ROBOT");
      }
    }
  }

  haz_cam_depth_to_image_trans.matrix() = M;
}

// Conventions used for the order of reading cameras
const int lua_nav_cam_type = 0;
const int lua_haz_cam_type = 1;

// This is a wrapper hiding some things
void readLuaConfig(bool & have_rig_transforms, int & ref_cam_type,
                    std::vector<std::string> & cam_names,
                    std::vector<camera::CameraParameters> & cam_params,
                    std::vector<Eigen::Affine3d> & ref_to_cam_trans,
                    std::vector<Eigen::Affine3d> & depth_to_image,
                    std::vector<double> & ref_to_cam_timestamp_offsets) {
  have_rig_transforms = true;
  ref_cam_type = lua_nav_cam_type;

  cam_names = {"nav_cam", "haz_cam", "sci_cam"};

  Eigen::Affine3d nav_cam_to_body_trans;
  Eigen::Affine3d haz_cam_depth_to_image;

  dense_map::readLuaConfig(  // Inputs
    cam_names, "nav_cam_transform", "haz_cam_depth_to_image_transform",
    // Outputs
    cam_params, ref_to_cam_trans, ref_to_cam_timestamp_offsets, nav_cam_to_body_trans,
    haz_cam_depth_to_image);

  // Depth to image transforms and scales
  depth_to_image.clear();
  depth_to_image.push_back(Eigen::Affine3d::Identity());  // nav
  depth_to_image.push_back(haz_cam_depth_to_image);      // haz
  depth_to_image.push_back(Eigen::Affine3d::Identity());  // sci
}

// Save some transforms from the robot calibration file. This has some very fragile
// logic and cannot handle comments in the config file.
void writeLuaConfig                                                           // NOLINT
(std::vector<std::string>              const& cam_names,                        // NOLINT
 std::string                           const& haz_cam_depth_to_image_trans_str, // NOLINT
 std::vector<camera::CameraParameters> const& cam_params,                       // NOLINT
 std::vector<Eigen::Affine3d>          const& nav_to_cam_trans,                 // NOLINT
 std::vector<double>                   const& nav_to_cam_timestamp_offset,      // NOLINT
 Eigen::Affine3d                       const& haz_cam_depth_to_image_trans) {   // NOLINT
  if (cam_names.size() != cam_params.size()  ||
      cam_names.size() != nav_to_cam_trans.size() ||
      cam_names.size() != nav_to_cam_timestamp_offset.size())
    LOG(FATAL) << "The number of various inputs to writeLuaConfig() do not match.";

  // Open the config file to modify
  char* config_dir = getenv("ASTROBEE_CONFIG_DIR");
  if (config_dir == NULL) LOG(FATAL) << "The environmental variable ASTROBEE_CONFIG_DIR was not set.";
  char* robot = getenv("ASTROBEE_ROBOT");
  if (robot == NULL) LOG(FATAL) << "The environmental variable ASTROBEE_ROBOT was not set.";
  std::string config_file = std::string(config_dir) + "/robots/" + robot + ".config";
  std::ifstream ifs(config_file.c_str());
  if (!ifs.is_open()) LOG(FATAL) << "Could not open file: " << config_file;

  // Read its text in one string
  std::string text;
  std::string line;
  while (std::getline(ifs, line)) text += line + "\n";
  ifs.close();

  std::cout << "Updating: " << config_file << std::endl;

  for (size_t it = 0; it < cam_names.size(); it++) {
    std::string              const& cam_name = cam_names[it];  // alias
    camera::CameraParameters const& params = cam_params[it];   // alias

    std::string intrinsics = intrinsics_to_string(params.GetFocalVector(),
                                                  params.GetOpticalOffset());
    replace_param_val("intrinsic_matrix", intrinsics, cam_name, "{", "}", text);
    Eigen::VectorXd cam_distortion = params.GetDistortion();

    // This can switch the distortion from being a number to being a vector,
    // and vice-versa, which makes the logic more complicated.
    std::string distortion_str;
    if (cam_distortion.size() > 1)
      distortion_str = vector_to_string(cam_distortion);
    else if (cam_distortion.size() == 1)
      distortion_str = number_to_string(cam_distortion[0]);
    else
      LOG(FATAL) << "Camera " << cam_name << " must have distortion.";

    // Note that we look at whether there are braces around param val
    // before replacement, rather than if the replacement param val
    // has braces.
    if (param_val_has_braces("distortion_coeff", cam_name, text))
      replace_param_val("distortion_coeff", distortion_str, cam_name, "{", "}", text);
    else
      replace_param_val("distortion_coeff", " " + distortion_str, cam_name, "", "", text);

    // Next deal with extrinsics

    if (cam_names[it] == "nav_cam") continue;  // this will have the trivial transforms

    std::string trans_name = "nav_cam_to_" + cam_names[it] + "_transform";
    std::string trans_val = affine_to_string(nav_to_cam_trans[it]);
    replace_param_val(trans_name, trans_val, "", "(", ")", text);

    std::string offset_str = "nav_cam_to_" + cam_names[it] + "_timestamp_offset";
    replace_param_val(offset_str, " " + number_to_string(nav_to_cam_timestamp_offset[it]), "",
                        "", "", text);
  }

  std::string depth_to_image_transform_val
    = matrix_to_string(haz_cam_depth_to_image_trans.matrix());
  replace_param_val(haz_cam_depth_to_image_trans_str, depth_to_image_transform_val,
                    "", "{", "}", text);

  // Write the updated values
  std::ofstream ofs(config_file.c_str());
  ofs << text;
  ofs.close();
}

// This is a wrapper for writeLuaConfig() hiding some things
void writeLuaConfig                                                        // NOLINT
(std::vector<std::string>              const& cam_names,                    // NOLINT
 std::vector<camera::CameraParameters> const& cam_params,                   // NOLINT
 std::vector<Eigen::Affine3d>          const& ref_to_cam_trans,             // NOLINT
 std::vector<double>                   const& ref_to_cam_timestamp_offset,  // NOLINT
 std::vector<Eigen::Affine3d>          const& depth_to_image_trans) {       // NOLINT
  // Update haz cam
  Eigen::Affine3d haz_cam_depth_to_image = depth_to_image_trans[lua_haz_cam_type];

  // Update the config file
  dense_map::writeLuaConfig(cam_names, "haz_cam_depth_to_image_transform",
                             cam_params, ref_to_cam_trans,
                             ref_to_cam_timestamp_offset,
                             haz_cam_depth_to_image);
}

}  // end namespace dense_map
