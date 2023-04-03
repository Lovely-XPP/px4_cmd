#ifndef HANDLE_CIN_H
#define HANDLE_CIN_H
#include <iostream>
#include <limits>

bool handle_cin(){
    bool state = std::cin.fail();
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return !state;
}
#endif