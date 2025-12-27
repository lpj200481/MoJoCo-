// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/simple_car/simple_car.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include <absl/random/random.h>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {

std::string SimpleCar::XmlPath() const {
  return GetModelPath("simple_car/task.xml");
}

std::string SimpleCar::Name() const { return "SimpleCar"; }

float speed = 0.0f;  // 当前车速（km/h）
void SimpleCar::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                     double* residual) const {
  residual[0] = data->qpos[0] - data->mocap_pos[0];
  residual[1] = data->qpos[1] - data->mocap_pos[1];
  residual[2] = data->ctrl[0];
  residual[3] = data->ctrl[1];
}

void SimpleCar::drawCircle(float radius, int segments) {}
void SimpleCar::drawTicks(float radius, int tickCount) {}
void SimpleCar::drawPointer(float angle) {}
void SimpleCar::drawNumber(float radius, int number, float angle) {}
void SimpleCar::drawDashboard(float* dashboard_pos, float speed_ratio) {}

void SimpleCar::TransitionLocked(mjModel* model, mjData* data) {
  double car_pos[2] = {data->qpos[0], data->qpos[1]};
  double goal_pos[2] = {data->mocap_pos[0], data->mocap_pos[1]};
  double car_to_goal[2];
  mju_sub(car_to_goal, goal_pos, car_pos, 2);

  if (mju_norm(car_to_goal, 2) < 0.2) {
    absl::BitGen gen_;
    data->mocap_pos[0] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[1] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[2] = 0.01;
  }
}

void SimpleCar::ModifyScene(const mjModel* model, const mjData* data,
                            mjvScene* scene) const {
  // ===== 油耗统计（累计）=====
  static double fuel_capacity = 100.0;  // 满油 = 100 单位
  static double fuel_used = 0.0;        // 累计油耗（任意单位）

  // 1) 位置
  double pos_x = data->qpos[0];
  double pos_y = data->qpos[1];

  // 2) 速度（qvel）
  double vel_x = data->qvel[0];
  double vel_y = data->qvel[1];

  // 3) 加速度
  double acc_x = data->qacc[0];
  double acc_y = data->qacc[1];

  // 4) 车体速度（用于转速模拟）
  double* car_veloc = SensorByName(model, data, "car_velocity");
  double speed_m = car_veloc ? mju_norm3(car_veloc) : 0.0;

  // 5) RPM 条（控制台）
  const int BAR_LEN = 30;
  const double max_speed_ref = 5.2;
  double rpm_ratio_bar = speed_m / max_speed_ref;
  if (rpm_ratio_bar > 1.0) rpm_ratio_bar = 1.0;
  if (rpm_ratio_bar < 0.0) rpm_ratio_bar = 0.0;

  int filled = static_cast<int>(rpm_ratio_bar * BAR_LEN);
  char rpm_bar[BAR_LEN + 1];
  for (int i = 0; i < BAR_LEN; i++) rpm_bar[i] = (i < filled) ? '#' : ' ';
  rpm_bar[BAR_LEN] = '\0';

  // 6) 油耗累积（按油门）
  double dt = model->opt.timestep;
  double throttle = data->ctrl[0];
  const double fuel_coeff = 0.18;

  fuel_used += fuel_coeff * std::abs(throttle) * dt;
  if (fuel_used > fuel_capacity) fuel_used = fuel_capacity;

  double fuel_left = fuel_capacity - fuel_used;
  double fuel_percent = (fuel_left / fuel_capacity) * 100.0;
  if (fuel_percent < 0.0) fuel_percent = 0.0;
  if (fuel_percent > 100.0) fuel_percent = 100.0;

  // 控制台打印
  std::printf(
      "\rPos(%.2f, %.2f) | Vel(%.2f, %.2f) | Acc(%.2f, %.2f) | Fuel %3.0f%% | "
      "RPM [%s]",
      pos_x, pos_y, vel_x, vel_y, acc_x, acc_y, fuel_percent, rpm_bar);
  std::fflush(stdout);

  // ===== 找到车体 =====
  int car_body_id = mj_name2id(model, mjOBJ_BODY, "car");
  if (car_body_id < 0) return;

  double* car_velocity = SensorByName(model, data, "car_velocity");
  if (!car_velocity) return;

  double speed_ms = mju_norm3(car_velocity);
  double speed_kmh = speed_ms * 3.6;

  double* car_pos = data->xpos + 3 * car_body_id;

  // ===== 速度盘比例 =====
  const float max_speed_kmh = 10.0f;
  float speed_ratio = static_cast<float>(speed_kmh) / max_speed_kmh;
  if (speed_ratio > 1.0f) speed_ratio = 1.0f;
  if (speed_ratio < 0.0f) speed_ratio = 0.0f;

  // ===== 转速盘比例（模拟）=====
  const float max_rpm = 8000.0f;
  float rpm_ratio = static_cast<float>(speed_m / max_speed_ref);
  if (rpm_ratio > 1.0f) rpm_ratio = 1.0f;
  if (rpm_ratio < 0.0f) rpm_ratio = 0.0f;
  float rpm_value = rpm_ratio * max_rpm;

  // ===== 仪表盘旋转（立起来 + 顺时针90度）=====
  double angle_x = 90.0 * 3.14159 / 180.0;
  double cos_x = std::cos(angle_x);
  double sin_x = std::sin(angle_x);
  double mat_x[9] = {1, 0, 0, 0, cos_x, -sin_x, 0, sin_x, cos_x};

  double angle_z = -90.0 * 3.14159 / 180.0;
  double cos_z = std::cos(angle_z);
  double sin_z = std::sin(angle_z);
  double mat_z[9] = {cos_z, -sin_z, 0, sin_z, cos_z, 0, 0, 0, 1};

  double dashboard_rot_mat[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dashboard_rot_mat[i * 3 + j] = 0;
      for (int k = 0; k < 3; k++) {
        dashboard_rot_mat[i * 3 + j] += mat_z[i * 3 + k] * mat_x[k * 3 + j];
      }
    }
  }

  // ===== 车体朝向：放到左右两侧 =====
  const double* car_xmat = data->xmat + 9 * car_body_id;
  double forward[3] = {car_xmat[0], car_xmat[3], car_xmat[6]};
  double left[3]    = {car_xmat[1], car_xmat[4], car_xmat[7]};
  double up[3]      = {car_xmat[2], car_xmat[5], car_xmat[8]};

  const float side_offset    = 0.22f;
  const float forward_offset = 0.10f;
  const float height_offset  = 0.20f;

  float base_pos[3] = {
      static_cast<float>(car_pos[0] + forward_offset * forward[0] + height_offset * up[0]),
      static_cast<float>(car_pos[1] + forward_offset * forward[1] + height_offset * up[1]),
      static_cast<float>(car_pos[2] + forward_offset * forward[2] + height_offset * up[2]),
  };

  float speed_dashboard_pos[3] = {
      base_pos[0] + side_offset * static_cast<float>(left[0]),
      base_pos[1] + side_offset * static_cast<float>(left[1]),
      base_pos[2] + side_offset * static_cast<float>(left[2]),
  };

  float rpm_dashboard_pos[3] = {
      base_pos[0] - side_offset * static_cast<float>(left[0]),
      base_pos[1] - side_offset * static_cast<float>(left[1]),
      base_pos[2] - side_offset * static_cast<float>(left[2]),
  };

  // ===== 半圆刻度角度（180° -> 0°，6个刻度）=====
  float tick_angles[6] = {180.0f, 144.0f, 108.0f, 72.0f, 36.0f, 0.0f};

  // ===== 绘制一个半圆仪表盘 =====
  auto DrawGauge = [&](const float center_pos[3], float ratio,
                       const int tick_values[6], float display_value,
                       const char* unit_text,
                       float pointer_r, float pointer_g, float pointer_b) {
    if (!scene) return;

    // 0) 半圆外弧（小段拼起来）
    const int ARC_SEG = 40;
    const float arc_radius = 0.15f;
    for (int s = 0; s <= ARC_SEG; s++) {
      if (scene->ngeom >= scene->maxgeom) break;

      float a = 180.0f - (180.0f * (float)s / (float)ARC_SEG);
      float rad = a * 3.14159f / 180.0f;

      mjvGeom* g = scene->geoms + scene->ngeom;
      g->type = mjGEOM_BOX;
      g->size[0] = 0.0025f;
      g->size[1] = 0.0045f;
      g->size[2] = 0.0025f;

      float y = center_pos[1] - arc_radius * std::cos(rad);
      float z = center_pos[2] + arc_radius * std::sin(rad);

      g->pos[0] = center_pos[0];
      g->pos[1] = y;
      g->pos[2] = z;

      for (int j = 0; j < 9; j++) g->mat[j] = static_cast<float>(dashboard_rot_mat[j]);

      g->rgba[0] = 0.7f;
      g->rgba[1] = 0.7f;
      g->rgba[2] = 0.7f;
      g->rgba[3] = 0.9f;
      scene->ngeom++;
    }

    // 1) 刻度线 + 数字
    for (int i = 0; i < 6; i++) {
      if (scene->ngeom >= scene->maxgeom) break;

      float rad_tick_angle = tick_angles[i] * 3.14159f / 180.0f;
      float tick_length = (i == 0 || i == 5 || i == 2) ? 0.02f : 0.015f;
      float tick_radius = 0.135f;

      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_BOX;
      geom->size[0] = 0.003f;
      geom->size[1] = tick_length;
      geom->size[2] = 0.003f;

      float tick_y = center_pos[1] - tick_radius * std::cos(rad_tick_angle);
      float tick_z = center_pos[2] + tick_radius * std::sin(rad_tick_angle);

      geom->pos[0] = center_pos[0];
      geom->pos[1] = tick_y;
      geom->pos[2] = tick_z;

      for (int j = 0; j < 9; j++) geom->mat[j] = static_cast<float>(dashboard_rot_mat[j]);

      geom->rgba[0] = 0.8f;
      geom->rgba[1] = 0.8f;
      geom->rgba[2] = 0.8f;
      geom->rgba[3] = 1.0f;
      scene->ngeom++;

      if (scene->ngeom < scene->maxgeom) {
        mjvGeom* label_geom = scene->geoms + scene->ngeom;
        label_geom->type = mjGEOM_LABEL;
        label_geom->size[0] = label_geom->size[1] = label_geom->size[2] = 0.05f;

        float label_radius = 0.18f;
        float label_y = center_pos[1] - label_radius * std::cos(rad_tick_angle);
        float label_z = center_pos[2] + label_radius * std::sin(rad_tick_angle);

        label_geom->pos[0] = center_pos[0];
        label_geom->pos[1] = label_y;
        label_geom->pos[2] = label_z;

        label_geom->rgba[0] = 0.85f;
        label_geom->rgba[1] = 0.85f;
        label_geom->rgba[2] = 0.85f;
        label_geom->rgba[3] = 1.0f;

        char tick_label[32];
        std::snprintf(tick_label, sizeof(tick_label), "%d", tick_values[i]);
        std::strncpy(label_geom->label, tick_label, sizeof(label_geom->label) - 1);
        label_geom->label[sizeof(label_geom->label) - 1] = '\0';
        scene->ngeom++;
      }
    }

    // 2) 指针（半圆：180 -> 0）
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_BOX;
      geom->size[0] = 0.004f;
      geom->size[1] = 0.11f;
      geom->size[2] = 0.003f;

      float angle = 180.0f - 180.0f * ratio;
      float rad_angle = angle * 3.14159f / 180.0f;

      float pointer_y = center_pos[1] - 0.055f * std::cos(rad_angle);
      float pointer_z = center_pos[2] + 0.055f * std::sin(rad_angle);

      geom->pos[0] = center_pos[0];
      geom->pos[1] = pointer_y;
      geom->pos[2] = pointer_z;

      double pointer_angle = angle - 90.0;
      double rad_pointer_angle = pointer_angle * 3.14159 / 180.0;
      double cos_p = std::cos(rad_pointer_angle);
      double sin_p = std::sin(rad_pointer_angle);
      double pointer_rot_mat[9] = {cos_p, -sin_p, 0, sin_p, cos_p, 0, 0, 0, 1};

      double temp_mat[9];
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          temp_mat[i * 3 + j] = 0;
          for (int k = 0; k < 3; k++) {
            temp_mat[i * 3 + j] += dashboard_rot_mat[i * 3 + k] *
                                   pointer_rot_mat[k * 3 + j];
          }
        }
      }
      for (int i = 0; i < 9; i++) geom->mat[i] = static_cast<float>(temp_mat[i]);

      geom->rgba[0] = pointer_r;
      geom->rgba[1] = pointer_g;
      geom->rgba[2] = pointer_b;
      geom->rgba[3] = 1.0f;
      scene->ngeom++;
    }

    // 3) 中心点
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_SPHERE;
      geom->size[0] = geom->size[1] = geom->size[2] = 0.006f;
      geom->pos[0] = center_pos[0];
      geom->pos[1] = center_pos[1];
      geom->pos[2] = center_pos[2];
      for (int j = 0; j < 9; j++) geom->mat[j] = static_cast<float>(dashboard_rot_mat[j]);
      geom->rgba[0] = 0.8f;
      geom->rgba[1] = 0.8f;
      geom->rgba[2] = 0.8f;
      geom->rgba[3] = 1.0f;
      scene->ngeom++;
    }

    // 4) 数字（中心偏上）
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_LABEL;
      geom->size[0] = geom->size[1] = geom->size[2] = 0.08f;
      geom->pos[0] = center_pos[0];
      geom->pos[1] = center_pos[1];
      geom->pos[2] = center_pos[2] + 0.02f;

      geom->rgba[0] = 0.92f;
      geom->rgba[1] = 0.92f;
      geom->rgba[2] = 0.92f;
      geom->rgba[3] = 1.0f;

      char value_label[64];
      std::snprintf(value_label, sizeof(value_label), "%.0f", display_value);
      std::strncpy(geom->label, value_label, sizeof(geom->label) - 1);
      geom->label[sizeof(geom->label) - 1] = '\0';
      scene->ngeom++;
    }

    // 5) 单位
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_LABEL;
      geom->size[0] = geom->size[1] = geom->size[2] = 0.05f;
      geom->pos[0] = center_pos[0];
      geom->pos[1] = center_pos[1];
      geom->pos[2] = center_pos[2] - 0.06f;

      geom->rgba[0] = 0.85f;
      geom->rgba[1] = 0.85f;
      geom->rgba[2] = 0.85f;
      geom->rgba[3] = 1.0f;

      std::strncpy(geom->label, unit_text, sizeof(geom->label) - 1);
      geom->label[sizeof(geom->label) - 1] = '\0';
      scene->ngeom++;
    }
  };

  const int speed_ticks[6] = {0, 3, 5, 7, 9, 11};
  const int rpm_ticks[6] = {0, 1500, 3000, 4500, 6000,7500};

  // 左：速度
  DrawGauge(speed_dashboard_pos, speed_ratio, speed_ticks,
            static_cast<float>(speed_kmh), "km/h",
            0.0f, 0.0f, 0.0f);

  // 右：转速
  DrawGauge(rpm_dashboard_pos, rpm_ratio, rpm_ticks,
            rpm_value, "RPM",
            0.0f, 1.0f, 0.0f);

  // ===== 中间显示 Fuel 剩余量（百分比）=====
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* mid = scene->geoms + scene->ngeom;
    mid->type = mjGEOM_LABEL;
    mid->size[0] = mid->size[1] = mid->size[2] = 0.06f;

    mid->pos[0] = base_pos[0];
    mid->pos[1] = base_pos[1];
    mid->pos[2] = base_pos[2] + 0.06f;

    mid->rgba[0] = 0.95f;
    mid->rgba[1] = 0.95f;
    mid->rgba[2] = 0.95f;
    mid->rgba[3] = 1.0f;

    char mid_label[64];
    std::snprintf(mid_label, sizeof(mid_label), "Fuel %.0f%%", fuel_percent);
    std::strncpy(mid->label, mid_label, sizeof(mid->label) - 1);
    mid->label[sizeof(mid->label) - 1] = '\0';

    scene->ngeom++;
  }
}

}  // namespace mjpc
