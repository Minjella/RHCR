#include "PBSNodeSection.h"
#include <iostream>

void PBSNodeSection::clear()
{
    conflicts.clear();
    priorities.clear();
}


void PBSNodeSection::print_priorities() const
{
    cout << "Priorities: ";
    for (auto row : priorities.G)
    {
        cout << row.first << " < (";
        for (auto a : row.second)
            std::cout << a << ", ";
        cout << "); ";
    }
    cout << endl;
}