#include "id_manager.hpp"

// 생성자 id_counter:256, id_notused 일련번호 대입
IDManager::IDManager()
{
    id_Counter = OBSTACLE_MAX;
    for (int i = 0; i < OBSTACLE_MAX; i++)
    {
        id_Notused[i] = OBSTACLE_MAX - i;
    }
}

int IDManager::allocID()
{
    return id_Notused[--id_Counter];
}

void IDManager::retID(int id)
{
    id_Notused[id_Counter++] = id;
}