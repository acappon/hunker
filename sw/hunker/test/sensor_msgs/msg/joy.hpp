#pragma once
#include <vector>
#include <memory>

namespace sensor_msgs { namespace msg {

struct Joy
{
    using SharedPtr = std::shared_ptr<Joy>;
    std::vector<float> axes;
    std::vector<int32_t> buttons;
};

struct JoyFeedback
{
    using SharedPtr = std::shared_ptr<JoyFeedback>;
    static constexpr int TYPE_RUMBLE = 1;
    int type = 0;
    int id = 0;
    float intensity = 0.0f;
};

}} // namespace sensor_msgs::msg
