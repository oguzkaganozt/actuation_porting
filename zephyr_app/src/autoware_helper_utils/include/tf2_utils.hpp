#include <cmath>
namespace tf2
{

class Quaternion
{
//TODO: define
  // tf2::Transform tf_pose;"
  // tf2::Transform tf_offset;
  // tf2::fromMsg(transform, tf_offset);
  // tf2::fromMsg(p, tf_pose);
  // tf2::toMsg(tf_pose * tf_offset, pose);"
public:
  Quaternion() : m_x(0), m_y(0), m_z(0), m_w(1) {}
  Quaternion(double x, double y, double z, double w) : m_x(x), m_y(y), m_z(z), m_w(w) {}

  double x() const { return m_x; }
  double y() const { return m_y; }
  double z() const { return m_z; }
  double w() const { return m_w; }

private:
  double m_x, m_y, m_z, m_w;
};

inline double getYaw(const Quaternion& q)
{
  double yaw;

  double sqw = q.w() * q.w();
  double sqx = q.x() * q.x();
  double sqy = q.y() * q.y();
  double sqz = q.z() * q.z();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x()*q.z() - q.w()*q.y()) / (sqx + sqy + sqz + sqw);

  if (sarg <= -0.99999) {
    yaw = -2 * std::atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    yaw = 2 * std::atan2(q.y(), q.x());
  } else {
    yaw = std::atan2(2 * (q.x()*q.y() + q.w()*q.z()), sqw + sqx - sqy - sqz);
  }
  return yaw;
}

} // namespace tf2
