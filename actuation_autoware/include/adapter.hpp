#include <vector>

namespace actuation::helper
{
    std::vector<double> to_vector(const geometry_msgs::msg::Point& point)
    {
        return std::vector<double>{point.x, point.y, point.z};
    }
}
