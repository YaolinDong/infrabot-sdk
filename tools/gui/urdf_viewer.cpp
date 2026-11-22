/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "urdf_viewer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include <QColor>
#include <QDomDocument>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLFunctions_2_1>
#include <QSurfaceFormat>
#include <QSet>
#include <QWheelEvent>

namespace infrabot_ik {
namespace {

constexpr double kPi = 3.14159265358979323846;

QVector3D numbers3(const QString& text, const QVector3D& fallback = QVector3D()) {
  const QStringList parts = text.simplified().split(' ', Qt::SkipEmptyParts);
  if (parts.size() != 3) return fallback;
  bool ok_x = false, ok_y = false, ok_z = false;
  const float x = parts[0].toFloat(&ok_x);
  const float y = parts[1].toFloat(&ok_y);
  const float z = parts[2].toFloat(&ok_z);
  return ok_x && ok_y && ok_z ? QVector3D(x, y, z) : fallback;
}

QMatrix4x4 originMatrix(const QDomElement& element) {
  QMatrix4x4 transform;
  if (element.isNull()) return transform;
  transform.translate(numbers3(element.attribute("xyz")));
  const QVector3D rpy = numbers3(element.attribute("rpy"));
  transform.rotate(static_cast<float>(rpy.z() * 180.0 / kPi), 0.0f, 0.0f, 1.0f);
  transform.rotate(static_cast<float>(rpy.y() * 180.0 / kPi), 0.0f, 1.0f, 0.0f);
  transform.rotate(static_cast<float>(rpy.x() * 180.0 / kPi), 1.0f, 0.0f, 0.0f);
  return transform;
}

struct Vertex {
  QVector3D position;
  QVector3D normal;
};

struct Mesh {
  QVector<Vertex> vertices;
  GLuint display_list{0};
};

struct Visual {
  QString link;
  QMatrix4x4 local;
  std::shared_ptr<Mesh> mesh;
  QColor color{205, 215, 225};
};

struct Joint {
  QString name;
  QString parent;
  QString child;
  QString type;
  QMatrix4x4 origin;
  QVector3D axis{0.0f, 0.0f, 1.0f};
  int index{-1};
};

float readFloat(const char* p) {
  float value = 0.0f;
  std::memcpy(&value, p, sizeof(value));
  return value;
}

QVector3D triangleNormal(const QVector3D& a, const QVector3D& b, const QVector3D& c) {
  const QVector3D n = QVector3D::crossProduct(b - a, c - a);
  return n.lengthSquared() > 1e-12f ? n.normalized() : QVector3D(0.0f, 0.0f, 1.0f);
}

std::shared_ptr<Mesh> loadStl(const QString& path, QString* error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("无法打开 STL：%1").arg(path);
    return {};
  }
  const QByteArray data = file.readAll();
  auto mesh = std::make_shared<Mesh>();
  if (data.size() >= 84) {
    const quint32 count = static_cast<quint8>(data[80]) |
                          (static_cast<quint32>(static_cast<quint8>(data[81])) << 8) |
                          (static_cast<quint32>(static_cast<quint8>(data[82])) << 16) |
                          (static_cast<quint32>(static_cast<quint8>(data[83])) << 24);
    const qint64 expected = 84LL + static_cast<qint64>(count) * 50LL;
    if (count > 0 && expected <= data.size()) {
      mesh->vertices.reserve(static_cast<int>(std::min<quint32>(count * 3U, 40000000U)));
      for (quint32 i = 0; i < count; ++i) {
        const char* tri = data.constData() + 84 + static_cast<qint64>(i) * 50;
        const QVector3D file_normal(readFloat(tri), readFloat(tri + 4), readFloat(tri + 8));
        QVector3D points[3] = {
            {readFloat(tri + 12), readFloat(tri + 16), readFloat(tri + 20)},
            {readFloat(tri + 24), readFloat(tri + 28), readFloat(tri + 32)},
            {readFloat(tri + 36), readFloat(tri + 40), readFloat(tri + 44)},
        };
        const QVector3D normal = file_normal.lengthSquared() > 1e-12f
                                     ? file_normal.normalized()
                                     : triangleNormal(points[0], points[1], points[2]);
        for (const QVector3D& point : points) mesh->vertices.push_back({point, normal});
      }
      return mesh;
    }
  }

  const QString text = QString::fromLatin1(data);
  QVector3D triangle[3];
  int point_count = 0;
  for (const QString& line : text.split('\n')) {
    const QString trimmed = line.trimmed();
    if (!trimmed.startsWith(QStringLiteral("vertex "))) continue;
    const QVector3D point = numbers3(trimmed.mid(7));
    triangle[point_count++] = point;
    if (point_count == 3) {
      const QVector3D normal = triangleNormal(triangle[0], triangle[1], triangle[2]);
      for (const QVector3D& p : triangle) mesh->vertices.push_back({p, normal});
      point_count = 0;
    }
  }
  if (mesh->vertices.isEmpty()) {
    if (error) *error = QStringLiteral("无法解析 STL：%1").arg(path);
    return {};
  }
  return mesh;
}

QColor materialColor(const QDomElement& visual) {
  const QDomElement color = visual.firstChildElement("material").firstChildElement("color");
  if (color.isNull()) return QColor(205, 215, 225);
  const QStringList values = color.attribute("rgba").simplified().split(' ', Qt::SkipEmptyParts);
  if (values.size() < 3) return QColor(205, 215, 225);
  bool ok[3] = {false, false, false};
  const int r = qBound(0, qRound(values[0].toDouble(&ok[0]) * 255.0), 255);
  const int g = qBound(0, qRound(values[1].toDouble(&ok[1]) * 255.0), 255);
  const int b = qBound(0, qRound(values[2].toDouble(&ok[2]) * 255.0), 255);
  return ok[0] && ok[1] && ok[2] ? QColor(r, g, b) : QColor(205, 215, 225);
}

}  // namespace

struct UrdfViewerWidget::Impl {
  QOpenGLFunctions_2_1* gl{nullptr};
  QVector<Visual> visuals;
  QVector<Joint> joints;
  QHash<QString, QVector<int>> children;
  QString root_link;
  QHash<QString, QMatrix4x4> link_poses;
  QPoint last_mouse;
  // Explicit spherical-camera angles. yaw=-45° places the camera in +X,-Y;
  // pitch=35° places it in +Z. paintGL() then uses lookAt(camera, center),
  // so the camera is unambiguously looking toward the robot.
  float yaw{-45.0f};
  float pitch{35.0f};
  float distance{0.75f};
  QVector3D center{0.0f, 0.0f, 0.18f};
  // Where resetView() returns (setHomeView()).
  float home_yaw{-45.0f};
  float home_pitch{35.0f};
  float home_distance{0.75f};
  QVector3D home_center{0.0f, 0.0f, 0.18f};
};

UrdfViewerWidget::UrdfViewerWidget(QWidget* parent)
    : QOpenGLWidget(parent), impl_(std::make_unique<Impl>()) {
  QSurfaceFormat format;
  format.setVersion(2, 1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);
  setFormat(format);
  setMinimumSize(680, 560);
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
}

UrdfViewerWidget::~UrdfViewerWidget() {
  if (context() && impl_->gl) {
    makeCurrent();
    releaseDisplayLists();
    doneCurrent();
  }
  delete impl_->gl;
  impl_->gl = nullptr;
}

void UrdfViewerWidget::releaseDisplayLists() {
  if (!impl_->gl) return;
  QSet<GLuint> lists;
  for (const Visual& visual : impl_->visuals) {
    if (visual.mesh && visual.mesh->display_list != 0) lists.insert(visual.mesh->display_list);
  }
  for (const GLuint list : lists) impl_->gl->glDeleteLists(list, 1);
  for (Visual& visual : impl_->visuals) {
    if (visual.mesh) visual.mesh->display_list = 0;
  }
}

void UrdfViewerWidget::buildDisplayLists() {
  if (!impl_->gl) return;
  for (Visual& visual : impl_->visuals) {
    if (!visual.mesh || visual.mesh->display_list != 0) continue;
    visual.mesh->display_list = impl_->gl->glGenLists(1);
    if (visual.mesh->display_list == 0) continue;
    impl_->gl->glNewList(visual.mesh->display_list, GL_COMPILE);
    impl_->gl->glBegin(GL_TRIANGLES);
    for (const Vertex& vertex : visual.mesh->vertices) {
      impl_->gl->glNormal3f(vertex.normal.x(), vertex.normal.y(), vertex.normal.z());
      impl_->gl->glVertex3f(vertex.position.x(), vertex.position.y(), vertex.position.z());
    }
    impl_->gl->glEnd();
    impl_->gl->glEndList();
  }
}

bool UrdfViewerWidget::loadUrdf(const QString& urdf_path, QString* error) {
  QFile file(urdf_path);
  if (!file.open(QIODevice::ReadOnly)) {
    last_error_ = QStringLiteral("无法打开 URDF：%1").arg(urdf_path);
    if (error) *error = last_error_;
    return false;
  }
  QDomDocument document;
  QString parse_error;
  int line = 0, column = 0;
  if (!document.setContent(&file, &parse_error, &line, &column)) {
    last_error_ = QStringLiteral("URDF XML 解析失败 %1:%2：%3").arg(line).arg(column).arg(parse_error);
    if (error) *error = last_error_;
    return false;
  }
  const QDomElement robot = document.documentElement();
  if (robot.tagName() != QStringLiteral("robot")) {
    last_error_ = QStringLiteral("URDF 根节点不是 robot");
    if (error) *error = last_error_;
    return false;
  }

  QVector<Joint> joints;
  QSet<QString> child_links;
  QSet<QString> all_links;
  for (QDomElement link = robot.firstChildElement("link"); !link.isNull(); link = link.nextSiblingElement("link")) {
    all_links.insert(link.attribute("name"));
  }
  for (QDomElement joint = robot.firstChildElement("joint"); !joint.isNull(); joint = joint.nextSiblingElement("joint")) {
    Joint parsed;
    parsed.name = joint.attribute("name");
    parsed.type = joint.attribute("type");
    parsed.parent = joint.firstChildElement("parent").attribute("link");
    parsed.child = joint.firstChildElement("child").attribute("link");
    parsed.origin = originMatrix(joint.firstChildElement("origin"));
    parsed.axis = numbers3(joint.firstChildElement("axis").attribute("xyz"), QVector3D(0.0f, 0.0f, 1.0f));
    if (parsed.type != QStringLiteral("fixed")) parsed.index = joints.size();
    joints.push_back(parsed);
    child_links.insert(parsed.child);
  }
  QString root_link;
  for (const QString& link : all_links) {
    if (!child_links.contains(link)) { root_link = link; break; }
  }
  if (root_link.isEmpty()) {
    last_error_ = QStringLiteral("URDF 没有找到根 link");
    if (error) *error = last_error_;
    return false;
  }

  QVector<Visual> visuals;
  QHash<QString, std::shared_ptr<Mesh>> mesh_cache;
  const QFileInfo urdf_info(urdf_path);
  for (QDomElement link = robot.firstChildElement("link"); !link.isNull(); link = link.nextSiblingElement("link")) {
    const QString link_name = link.attribute("name");
    for (QDomElement visual = link.firstChildElement("visual"); !visual.isNull(); visual = visual.nextSiblingElement("visual")) {
      const QDomElement mesh_element = visual.firstChildElement("geometry").firstChildElement("mesh");
      if (mesh_element.isNull()) continue;
      QString mesh_path = mesh_element.attribute("filename");
      QDir mesh_dir = urdf_info.dir();
      const QString package_prefix = QStringLiteral("package://infrabot_description/");
      if (mesh_path.startsWith(package_prefix)) {
        mesh_dir.cdUp();  // <package>/urdf/infrabot_h7c.urdf
        mesh_path.remove(0, package_prefix.size());
      }
      const QString resolved = mesh_dir.absoluteFilePath(mesh_path);
      std::shared_ptr<Mesh> mesh = mesh_cache.value(resolved);
      if (!mesh) {
        QString mesh_error;
        mesh = loadStl(resolved, &mesh_error);
        if (!mesh) {
          last_error_ = mesh_error;
          if (error) *error = last_error_;
          return false;
        }
        mesh_cache.insert(resolved, mesh);
      }
      QMatrix4x4 local = originMatrix(visual.firstChildElement("origin"));
      const QVector3D scale = numbers3(mesh_element.attribute("scale"), QVector3D(1.0f, 1.0f, 1.0f));
      local.scale(scale);
      visuals.push_back({link_name, local, mesh, materialColor(visual)});
    }
  }
  if (visuals.isEmpty()) {
    last_error_ = QStringLiteral("URDF 没有可渲染的 visual mesh");
    if (error) *error = last_error_;
    return false;
  }

  const bool had_context = context() != nullptr;
  if (had_context) makeCurrent();
  if (impl_->gl) releaseDisplayLists();
  impl_->visuals = std::move(visuals);
  impl_->joints = std::move(joints);
  impl_->root_link = root_link;
  impl_->children.clear();
  for (int i = 0; i < impl_->joints.size(); ++i) impl_->children[impl_->joints[i].parent].push_back(i);
  urdf_path_ = QFileInfo(urdf_path).absoluteFilePath();
  q_.fill(0.0);
  if (impl_->gl) buildDisplayLists();
  if (had_context) doneCurrent();
  update();
  if (error) error->clear();
  last_error_.clear();
  return true;
}

void UrdfViewerWidget::setJointValues(const std::array<double, 7>& q) {
  q_ = q;
  update();
}

void UrdfViewerWidget::resetView() {
  impl_->yaw = impl_->home_yaw;
  impl_->pitch = impl_->home_pitch;
  impl_->distance = impl_->home_distance;
  impl_->center = impl_->home_center;
  update();
}

void UrdfViewerWidget::setHomeView(float yaw_deg, float pitch_deg, float distance_m,
                                   const QVector3D& center) {
  impl_->home_yaw = yaw_deg;
  impl_->home_pitch = qBound(-89.0f, pitch_deg, 89.0f);
  impl_->home_distance = qBound(0.08f, distance_m, 8.0f);
  impl_->home_center = center;
  resetView();
}

void UrdfViewerWidget::initializeGL() {
  if (impl_->gl) {
    for (Visual& visual : impl_->visuals) {
      if (visual.mesh) visual.mesh->display_list = 0;
    }
  }
  delete impl_->gl;
  impl_->gl = new QOpenGLFunctions_2_1;
  if (!impl_->gl->initializeOpenGLFunctions()) {
    delete impl_->gl;
    impl_->gl = nullptr;
    return;
  }
  impl_->gl->glEnable(GL_DEPTH_TEST);
  impl_->gl->glEnable(GL_CULL_FACE);
  impl_->gl->glEnable(GL_COLOR_MATERIAL);
  impl_->gl->glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  impl_->gl->glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
  buildDisplayLists();
}

void UrdfViewerWidget::resizeGL(int width, int height) {
  if (!impl_->gl) return;
  const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
  QMatrix4x4 projection;
  projection.perspective(45.0f, aspect, 0.005f, 20.0f);
  impl_->gl->glMatrixMode(GL_PROJECTION);
  impl_->gl->glLoadMatrixf(projection.constData());
}

void UrdfViewerWidget::paintGL() {
  if (!impl_->gl) return;
  impl_->gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  QMatrix4x4 view;
  const float yaw = impl_->yaw * static_cast<float>(kPi / 180.0);
  const float pitch = impl_->pitch * static_cast<float>(kPi / 180.0);
  const float horizontal = std::cos(pitch) * impl_->distance;
  const QVector3D camera = impl_->center + QVector3D(
      std::cos(yaw) * horizontal,
      std::sin(yaw) * horizontal,
      std::sin(pitch) * impl_->distance);
  // camera = (+X, -Y, +Z) for the reset angles above, and always looks at
  // the model center rather than looking along an inferred inverse rotation.
  view.lookAt(camera, impl_->center, QVector3D(0.0f, 0.0f, 1.0f));
  impl_->gl->glMatrixMode(GL_MODELVIEW);
  impl_->gl->glLoadMatrixf(view.constData());

  impl_->gl->glDisable(GL_LIGHTING);
  impl_->gl->glLineWidth(1.0f);
  impl_->gl->glBegin(GL_LINES);
  for (int i = -10; i <= 10; ++i) {
    const float p = static_cast<float>(i) * 0.1f;
    impl_->gl->glColor3f(0.25f, 0.25f, 0.28f);
    impl_->gl->glVertex3f(-1.0f, p, 0.0f); impl_->gl->glVertex3f(1.0f, p, 0.0f);
    impl_->gl->glVertex3f(p, -1.0f, 0.0f); impl_->gl->glVertex3f(p, 1.0f, 0.0f);
  }
  impl_->gl->glColor3f(0.85f, 0.15f, 0.15f); impl_->gl->glVertex3f(0, 0, 0); impl_->gl->glVertex3f(0.25f, 0, 0);
  impl_->gl->glColor3f(0.15f, 0.85f, 0.25f); impl_->gl->glVertex3f(0, 0, 0); impl_->gl->glVertex3f(0, 0.25f, 0);
  impl_->gl->glColor3f(0.15f, 0.35f, 0.95f); impl_->gl->glVertex3f(0, 0, 0); impl_->gl->glVertex3f(0, 0, 0.25f);
  impl_->gl->glEnd();

  impl_->link_poses.clear();
  impl_->link_poses.insert(impl_->root_link, QMatrix4x4());
  QVector<int> pending = impl_->children.value(impl_->root_link);
  while (!pending.isEmpty()) {
    const int joint_index = pending.takeFirst();
    const Joint& joint = impl_->joints[joint_index];
    QMatrix4x4 pose = impl_->link_poses.value(joint.parent);
    pose *= joint.origin;
    if (joint.index >= 0 && joint.index < 7) {
      pose.rotate(static_cast<float>(q_[joint.index] * 180.0 / kPi), joint.axis);
    }
    impl_->link_poses.insert(joint.child, pose);
    pending += impl_->children.value(joint.child);
  }

  impl_->gl->glEnable(GL_LIGHTING);
  impl_->gl->glEnable(GL_LIGHT0);
  const GLfloat light_position[] = {1.5f, 1.5f, 2.0f, 1.0f};
  impl_->gl->glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  const GLfloat diffuse[] = {0.9f, 0.9f, 0.9f, 1.0f};
  impl_->gl->glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  for (const Visual& visual : impl_->visuals) {
    const QMatrix4x4 world = impl_->link_poses.value(visual.link) * visual.local;
    if (!visual.mesh || visual.mesh->display_list == 0) continue;
    impl_->gl->glPushMatrix();
    impl_->gl->glMultMatrixf(world.constData());
    impl_->gl->glColor3f(visual.color.redF(), visual.color.greenF(), visual.color.blueF());
    impl_->gl->glCallList(visual.mesh->display_list);
    impl_->gl->glPopMatrix();
  }
}

void UrdfViewerWidget::mousePressEvent(QMouseEvent* event) {
  impl_->last_mouse = event->pos();
  event->accept();
}

void UrdfViewerWidget::mouseMoveEvent(QMouseEvent* event) {
  const QPoint delta = event->pos() - impl_->last_mouse;
  impl_->last_mouse = event->pos();
  if (event->buttons() & Qt::LeftButton) {
    // Screen-right drag should orbit the camera to the user's right. The
    // spherical camera uses a world azimuth, so its sign is opposite to the
    // raw screen X delta.
    impl_->yaw -= static_cast<float>(delta.x()) * 0.5f;
    impl_->pitch = qBound(-89.0f, impl_->pitch + static_cast<float>(delta.y()) * 0.5f, 89.0f);
    update();
  }
  event->accept();
}

void UrdfViewerWidget::wheelEvent(QWheelEvent* event) {
  impl_->distance = qBound(0.08f, impl_->distance * std::pow(0.85f, event->angleDelta().y() / 120.0f), 8.0f);
  update();
  event->accept();
}

}  // namespace infrabot_ik
