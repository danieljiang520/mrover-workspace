
#include <iostream>
#include <string>

#include "a.h"
#include "b.h"
using namespace std;
int main() {
    A a;
    B b;
    b.p = 6;
    a.a = 3;
    run(&a);
    run2(&b);
}

void run(A* ref) {
    cout << "A is " << ref->a << "\n";
}
void run2(B* ref) {
    cout << "B is " << ref->p << "\n";
}