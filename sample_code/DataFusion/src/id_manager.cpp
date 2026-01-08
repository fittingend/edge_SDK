#include "id_manager.hpp"

// 생성자: 풀을 1~OBSTACLE_MAX까지 역순으로 채워 스택처럼 사용
IDManager::IDManager()
{
    pool_.reserve(OBSTACLE_MAX);
    for (int i = OBSTACLE_MAX; i >= 1; --i)
    {
        pool_.push_back(static_cast<std::uint16_t>(i));
    }
    top_ = pool_.size();
}

int IDManager::allocID()
{
    if (top_ == 0)
    {
        return -1; // exhausted
    }
    return pool_[--top_];
}

void IDManager::retID(int id)
{
    if (id <= 0 || id > OBSTACLE_MAX)
        return; // ignore invalid
    if (top_ < pool_.size())
    {
        pool_[top_++] = static_cast<std::uint16_t>(id);
    }
}

int IDManager::getNum()
{
    return static_cast<int>(pool_.size() - top_);
}