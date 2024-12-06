#ifndef __IDMANAGER_HPP__
#define __IDMANAGER_HPP__

#include <iostream>
#include <string>

#define OBSTACLE_MAX 256


class IDManager
{
public:
    IDManager();
    int allocID();
    void retID(int id);

private:
    std::uint16_t id_Counter;
    std::uint8_t id_Notused[OBSTACLE_MAX];
};
#endif