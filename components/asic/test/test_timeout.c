#include "unity.h"

#include "asic_common.h"

TEST_CASE("Check asic timeout 1x BM1397", "[common]")
{
    float frequency = 450.0; // MHz
    size_t asic_count = 1;
    size_t small_cores = 672;
    size_t cores = 168;
    size_t version_size = 4;
    float timeout_percent = 0.75;
    float default_timeout_ms = 20;

    double timeout_ms = calculate_bm_timeout_ms(frequency, asic_count, small_cores, cores, version_size, timeout_percent, default_timeout_ms);
    double expected_ms = 27.962;

    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_ms, timeout_ms);
}

TEST_CASE("Check asic timeout 2x BM1370", "[common]")
{
    float frequency = 450.0; // MHz
    size_t asic_count = 2;
    size_t small_cores = 2040;
    size_t cores = 128;
    size_t version_size = 65536;
    float timeout_percent = 0.5;
    float default_timeout_ms = 500;

    double timeout_ms = calculate_bm_timeout_ms(frequency, asic_count, small_cores, cores, version_size, timeout_percent, default_timeout_ms);
    double expected_ms = 76354.974;

    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_ms, timeout_ms);
}

TEST_CASE("Check default asic timeout 0x BM1370", "[common]")
{
    float frequency = 450.0; // MHz
    size_t asic_count = 0; // 0 chip example
    size_t small_cores = 2040;
    size_t cores = 128;
    size_t version_size = 65536;
    float timeout_percent = 0.5;
    float default_timeout_ms = 500;

    double timeout_ms = calculate_bm_timeout_ms(frequency, asic_count, small_cores, cores, version_size, timeout_percent, default_timeout_ms);

    TEST_ASSERT_FLOAT_WITHIN(0.01, default_timeout_ms, timeout_ms);
}

TEST_CASE("Check asic timeout 3x BM1370", "[common]")
{
    float frequency = 450.0; // MHz
    size_t asic_count = 3; // not power of 2 chain length
    size_t small_cores = 2040;
    size_t cores = 128;
    size_t version_size = 256;
    float timeout_percent = 0.5;
    float default_timeout_ms = 500;

    double timeout_ms = calculate_bm_timeout_ms(frequency, asic_count, small_cores, cores, version_size, timeout_percent, default_timeout_ms);
    double expected_ms = 149.131;

    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_ms, timeout_ms);
}

TEST_CASE("Check max asic timeout 1x BM1370", "[common]")
{
    float frequency = 450.0; // MHz
    size_t asic_count = 1;
    size_t small_cores = 2040;
    size_t cores = 128;
    size_t version_size = 65536;
    float timeout_percent = 1.0;
    float default_timeout_ms = 500;

    double timeout_ms = calculate_bm_timeout_ms(frequency, asic_count, small_cores, cores, version_size, timeout_percent, default_timeout_ms);
    double expected_ms = 305419.897;

    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_ms, timeout_ms);
}

TEST_CASE("HCN calculation matches BM1366 defaults", "[common]")
{
    uint32_t hcn = calculate_bm_hcn(485.0f, 1, 112, 1.0, 25.0, 0.0);
    TEST_ASSERT_EQUAL_UINT32(864804, hcn);
}

TEST_CASE("HCN calculation applies BM1370 correction", "[common]")
{
    uint32_t hcn = calculate_bm_hcn(525.0f, 2, 128, 1.0, 25.0, 268.0);
    TEST_ASSERT_EQUAL_UINT32(399189, hcn);
}

TEST_CASE("HCN calculation rejects invalid frequency", "[common]")
{
    TEST_ASSERT_EQUAL_UINT32(0, calculate_bm_hcn(0.0f, 1, 128, 1.0, 25.0, 0.0));
}

TEST_CASE("Version-rolling job interval preserves refresh cap", "[common]")
{
    double interval_ms = calculate_bm_job_interval_ms(
        525.0f, 2, 2040, 128, 65536, 250.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 250.0, interval_ms);
}

TEST_CASE("Job interval never exceeds calculated scan time", "[common]")
{
    double interval_ms = calculate_bm_job_interval_ms(
        450.0f, 1, 672, 168, 4, 1000.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 37.282, interval_ms);
}
