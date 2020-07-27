/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <string>

namespace basalt {

struct VioConfig {
  VioConfig();
  void load(const std::string& filename);
  void save(const std::string& filename);

  std::string optical_flow_type = "frame_to_frame";
  int optical_flow_detection_grid_size = 50;
  float optical_flow_max_recovered_dist2 = 0.09f;
  int optical_flow_pattern = 51;
  int optical_flow_max_iterations = 5;
  int optical_flow_levels = 3;
  float optical_flow_epipolar_error = 0.005;
  int optical_flow_skip_frames = 1;

  int vio_max_states = 3;
  int vio_max_kfs = 7;
  int vio_min_frames_after_kf = 5;
  float vio_new_kf_keypoints_thresh = 0.7;
  bool vio_debug = false;

  double vio_outlier_threshold = 3.0;
  int vio_filter_iteration = 4;
  int vio_max_iterations = 7;

  double vio_obs_std_dev = 0.5;
  double vio_obs_huber_thresh = 1.0;
  double vio_min_triangulation_dist = 0.05;

  bool vio_enforce_realtime = 3.0;

  bool vio_use_lm = false;
  double vio_lm_lambda_min = 1e-32;
  double vio_lm_lambda_max = 1e2;

  double vio_init_pose_weight = 1e8;
  double vio_init_ba_weight = 1e1;
  double vio_init_bg_weight = 1e2;

  double mapper_obs_std_dev = 0.25;
  double mapper_obs_huber_thresh = 1.5;
  int mapper_detection_num_points = 800;
  double mapper_num_frames_to_match = 30;
  double mapper_frames_to_match_threshold = 0.04;
  double mapper_min_matches = 20;
  double mapper_ransac_threshold = 5e-5;
  double mapper_min_track_length = 5;
  double mapper_max_hamming_distance = 70;
  double mapper_second_best_test_ratio = 1.2;
  int mapper_bow_num_bits = 16;
  double mapper_min_triangulation_dist = 0.07;
  bool mapper_no_factor_weights = false;
  bool mapper_use_factors = true;

  bool mapper_use_lm = true;
  double mapper_lm_lambda_min = 1e-32;
  double mapper_lm_lambda_max = 1e2;
};
}  // namespace basalt
