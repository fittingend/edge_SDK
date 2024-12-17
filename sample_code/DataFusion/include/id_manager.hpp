#ifndef __IDMANAGER_HPP__
#define __IDMANAGER_HPP__

#include <iostream>
#include <string>

#define OBSTACLE_MAX 1000

class IDManager
{
public:
    IDManager();
    int allocID();
    void retID(int id);
    int getNum();

private:
    std::uint16_t id_Counter;
    std::uint8_t id_Notused[OBSTACLE_MAX];
};
#endif