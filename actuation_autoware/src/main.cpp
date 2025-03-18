// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <unistd.h>

int a = 0;
int main(void)
{   
    while (1) {
        a++;
        int c = a;
        c++;
        printf("Hello World\n");
        printf("a: %d\n", c);
        printf("a: %d\n", a);
        sleep(1);
    }

    return 0;
}
