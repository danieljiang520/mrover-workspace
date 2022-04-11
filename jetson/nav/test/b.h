#pragma once
struct B;
#include "a.h"

struct B {
    int p;
    double q;
    A* ref;
};

void run2(B* ref);