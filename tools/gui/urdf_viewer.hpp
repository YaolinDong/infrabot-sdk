/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

#include <array>
#include <memory>
#include <QString>
#include <QVector>
#include <QVector3D>
#include <QOpenGLWidget>

class QMouseEvent;
class QWheelEvent;

namespace infrabot_ik {

class UrdfViewerWidget final : public QOpenGLWidget {
 public:
  explicit UrdfViewerWidget(QWidget* parent = nullptr);
  ~UrdfViewerWidget() override;

  bool loadUrdf(const QString& urdf_path, QString* error = nullptr);
  QString urdfPath() const { return urdf_path_; }
  QString lastError() const { return last_error_; }

  void setJointValues(const std::array<double, 7>& q);
  std::array<double, 7> jointValues() const { return q_; }
  void resetView();
  // The view resetView() returns to, applied at once: camera azimuth about
  // +Z from +X and elevation in degrees, distance in metres, looking at
  // center.  Embedding windows whose viewer is smaller than this UI's use it
  // to keep the whole arm in view.
  void setHomeView(float yaw_deg, float pitch_deg, float distance_m, const QVector3D& center);

 protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  void releaseDisplayLists();
  void buildDisplayLists();

  struct Impl;
  std::unique_ptr<Impl> impl_;
  QString urdf_path_;
  QString last_error_;
  std::array<double, 7> q_{};
};

}  // namespace infrabot_ik
