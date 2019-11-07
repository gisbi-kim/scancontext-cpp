#ifndef _UTILS_H_
#define _UTILS_H_

#include <iostream>
#include <vector>
#include <string>

#include "scancontext.h"

using namespace std;

namespace utils
{

template<class T>
void disp(T msg_)
{
    cout << msg_ << endl;
}

void dispInt (int& msg_);


} // namespace utils 
#endif // _UTILS_H_

