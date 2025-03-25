// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <unistd.h>

int main(void)
{   
    printf("Hello World\n");
    
    while (1) {
        sleep(1);
    }

    return 0;
}
