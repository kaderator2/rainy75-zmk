#ifndef LL_TEST_H
#define LL_TEST_H
#include <stdio.h>
#include <stdlib.h>
static int ll_tests_failed = 0;
#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ll_tests_failed++; } } while (0)
#define DONE() do { if (ll_tests_failed) { printf("%d FAILED\n", ll_tests_failed); \
    return 1; } printf("OK\n"); return 0; } while (0)
#endif
