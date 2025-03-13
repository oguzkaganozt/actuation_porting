// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <unistd.h>

int main(void)
{   
    while (1) {
        fprintf(stderr, "Hello World\n");
        fprintf(stderr, "This is actuation autoware\n");
        sleep(1);
    }

    return 0;
}
