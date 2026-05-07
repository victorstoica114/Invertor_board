#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int g_unity_failures = 0;
static int g_unity_tests_run = 0;

static inline void unity_fail_impl(const char *file, int line, const char *expr, const char *msg)
{
    g_unity_failures++;
    fprintf(stderr, "FAIL %s:%d: %s", file, line, expr);
    if (msg != NULL) {
        fprintf(stderr, " (%s)", msg);
    }
    fprintf(stderr, "\n");
}

#define UNITY_BEGIN() (g_unity_failures = 0, g_unity_tests_run = 0)
#define UNITY_END() (g_unity_failures)

#define RUN_TEST(fn) \
    do { \
        extern void setUp(void); \
        extern void tearDown(void); \
        g_unity_tests_run++; \
        setUp(); \
        fn(); \
        tearDown(); \
    } while (0)

#define TEST_ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) unity_fail_impl(__FILE__, __LINE__, #condition, NULL); \
    } while (0)

#define TEST_ASSERT_FALSE(condition) TEST_ASSERT_TRUE(!(condition))

#define TEST_ASSERT_EQUAL(expected, actual) \
    do { \
        long long e_ = (long long)(expected); \
        long long a_ = (long long)(actual); \
        if (e_ != a_) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_EQUAL", NULL); \
    } while (0)

#define TEST_ASSERT_EQUAL_UINT8(expected, actual) \
    do { \
        uint8_t e_ = (uint8_t)(expected); \
        uint8_t a_ = (uint8_t)(actual); \
        if (e_ != a_) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_EQUAL_UINT8", NULL); \
    } while (0)

#define TEST_ASSERT_EQUAL_UINT16(expected, actual) \
    do { \
        uint16_t e_ = (uint16_t)(expected); \
        uint16_t a_ = (uint16_t)(actual); \
        if (e_ != a_) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_EQUAL_UINT16", NULL); \
    } while (0)

#define TEST_ASSERT_EQUAL_INT16(expected, actual) \
    do { \
        int16_t e_ = (int16_t)(expected); \
        int16_t a_ = (int16_t)(actual); \
        if (e_ != a_) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_EQUAL_INT16", NULL); \
    } while (0)

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
    do { \
        float d_ = (float)(actual) - (float)(expected); \
        if (d_ < 0.0f) d_ = -d_; \
        if (d_ > (float)(delta)) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_FLOAT_WITHIN", NULL); \
    } while (0)

#define TEST_ASSERT_EQUAL_UINT32(expected, actual) \
    do { \
        uint32_t e_ = (uint32_t)(expected); \
        uint32_t a_ = (uint32_t)(actual); \
        if (e_ != a_) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_EQUAL_UINT32", NULL); \
    } while (0)

#define TEST_ASSERT_EQUAL_PTR_MESSAGE(expected, actual, msg) \
    do { \
        if ((expected) != (actual)) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_EQUAL_PTR_MESSAGE", (msg)); \
    } while (0)

#define TEST_ASSERT_EQUAL_HEX16(expected, actual) TEST_ASSERT_EQUAL_UINT16((expected), (actual))

#define TEST_ASSERT_GREATER_THAN_UINT8(threshold, actual) \
    do { \
        if (!((uint8_t)(actual) > (uint8_t)(threshold))) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_GREATER_THAN_UINT8", NULL); \
    } while (0)

#define TEST_ASSERT_NOT_EQUAL(expected, actual) \
    do { \
        if ((expected) == (actual)) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_NOT_EQUAL", NULL); \
    } while (0)

#define TEST_ASSERT_EQUAL_STRING(expected, actual) \
    do { \
        const char *e_ = (expected); \
        const char *a_ = (actual); \
        if ((e_ == NULL && a_ != NULL) || (e_ != NULL && a_ == NULL) || (e_ != NULL && a_ != NULL && strcmp(e_, a_) != 0)) { \
            unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_EQUAL_STRING", NULL); \
        } \
    } while (0)

#define TEST_ASSERT_GREATER_THAN_UINT16(threshold, actual) \
    do { \
        if (!((uint16_t)(actual) > (uint16_t)(threshold))) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_GREATER_THAN_UINT16", NULL); \
    } while (0)

#define TEST_ASSERT_LESS_OR_EQUAL_UINT8(threshold, actual) \
    do { \
        if (!((uint8_t)(actual) <= (uint8_t)(threshold))) unity_fail_impl(__FILE__, __LINE__, "TEST_ASSERT_LESS_OR_EQUAL_UINT8", NULL); \
    } while (0)
