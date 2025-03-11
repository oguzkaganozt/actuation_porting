// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <unistd.h>

int main(void)
{   
    while (1) {
        printf("Hello World\n");
        printf("Hello World2\n");
        printf("This is actuation autoware\n");
        printf("This is actuation autoware2\n");
        sleep(1);
    }

    return 0;
}
