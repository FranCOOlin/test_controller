#ifndef COMMON_STATE_H
#define COMMON_STATE_H

namespace common {

class State {
public:
    bool updated;  // 标记状态是否更新

    State() : updated(false) { }
    virtual ~State() = default;
};

} // namespace common

#endif // COMMON_STATE_H
