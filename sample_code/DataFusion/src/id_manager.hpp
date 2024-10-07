#ifndef __IDMANAGER_HPP__
#define __IDMANAGER_HPP__

#include <iostream>
#include <string>
#include "main_datafusion.hpp"

#define OBSTACLE_MAX 256

using namespace std;

class IDManager
{
public:
    IDManager();
    int allocID();
    void retID(int id);

private:
    uint16_t id_Counter;
    uint8_t id_Notused[OBSTACLE_MAX];
};
#endif