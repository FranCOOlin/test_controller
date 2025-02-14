#ifndef COMMON_CONTROL_INPUT_H
#define COMMON_CONTROL_INPUT_H

namespace common {

class ControlInput {
public:
    bool updated;  // 标记控制输入是否更新

    ControlInput() : updated(false) { }
    virtual ~ControlInput() = default;

};

} // namespace common

#endif // COMMON_CONTROL_INPUT_H
