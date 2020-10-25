//
//  main.cpp
//  PSO
//
//  Created by TzuChieh on 2020/07/29
//  Copyright © 2020 TzuChieh. All rights reserved.
//

#include "function.hpp"

int main(int argc, const char * argv[]) {
    srand((unsigned int)time(NULL));
    int ITE = atoi(argv[1]);
    int RUN = atoi(argv[2]);
    PSO_RUN(ITE , RUN);
}
