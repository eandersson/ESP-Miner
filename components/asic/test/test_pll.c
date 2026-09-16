#include "unity.h"

#include <math.h>

#include "pll.h"

TEST_CASE("Check PLL frequency calculation", "[pll]")
{
    float frequency = 450.0; // MHz
    uint8_t fb_divider, refdiv, postdiv1, postdiv2;
    float actual_freq;

    TEST_ASSERT_EQUAL(ESP_OK,
                      pll_get_parameters(frequency, 60, 200, &fb_divider,
                                         &refdiv, &postdiv1, &postdiv2,
                                         &actual_freq));

    TEST_ASSERT_EQUAL_UINT8(72, fb_divider);
    TEST_ASSERT_EQUAL_UINT8(2, refdiv);
    TEST_ASSERT_EQUAL_UINT8(2, postdiv1);
    TEST_ASSERT_EQUAL_UINT8(1, postdiv2);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 450.0, actual_freq);
}

TEST_CASE("PLL calculation rejects invalid inputs without modifying outputs", "[pll]")
{
    uint8_t fb_divider = 0xAA;
    uint8_t refdiv = 0xBB;
    uint8_t postdiv1 = 0xCC;
    uint8_t postdiv2 = 0xDD;
    float actual_freq = 123.0f;

    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        pll_get_parameters(NAN, 60, 200, &fb_divider, &refdiv, &postdiv1,
                           &postdiv2, &actual_freq));
    TEST_ASSERT_EQUAL_HEX8(0xAA, fb_divider);
    TEST_ASSERT_EQUAL_HEX8(0xBB, refdiv);
    TEST_ASSERT_EQUAL_HEX8(0xCC, postdiv1);
    TEST_ASSERT_EQUAL_HEX8(0xDD, postdiv2);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 123.0f, actual_freq);

    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        pll_get_parameters(450.0f, 200, 60, &fb_divider, &refdiv,
                           &postdiv1, &postdiv2, &actual_freq));
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        pll_get_parameters(450.0f, 60, 300, &fb_divider, &refdiv,
                           &postdiv1, &postdiv2, &actual_freq));

    TEST_ASSERT_EQUAL(
        ESP_ERR_NOT_FOUND,
        pll_get_parameters(1.0f, 60, 200, &fb_divider, &refdiv,
                           &postdiv1, &postdiv2, &actual_freq));
    TEST_ASSERT_EQUAL_HEX8(0xAA, fb_divider);
    TEST_ASSERT_EQUAL_HEX8(0xBB, refdiv);
    TEST_ASSERT_EQUAL_HEX8(0xCC, postdiv1);
    TEST_ASSERT_EQUAL_HEX8(0xDD, postdiv2);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 123.0f, actual_freq);
}
