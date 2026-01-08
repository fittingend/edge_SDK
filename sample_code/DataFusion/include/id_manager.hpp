#ifndef __IDMANAGER_HPP__
#define __IDMANAGER_HPP__

#include <iostream>
#include <string>
#include <vector>

#define OBSTACLE_MAX 65000

class IDManager
{
public:
    IDManager();
    int allocID();
    void retID(int id);
    int getNum();

private:
    std::size_t top_;
    std::vector<std::uint16_t> pool_;
};
#endif