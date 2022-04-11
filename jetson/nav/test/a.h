#pragma once
struct A;

#include "b.h"

struct A{
    int a;
    double b;
    B* ref;
};

void run(A* ref);