#ifndef COMMON_MEASUREMENT_H
#define COMMON_MEASUREMENT_H

namespace common {

class Measurement {
public:
    bool updated;  // 标记测量是否更新

    // 构造函数：初始化测量值和更新标志
    Measurement() : updated(false) { }

    // 析构函数
    virtual ~Measurement() = default;
};

} // namespace common

#endif // COMMON_MEASUREMENT_H
