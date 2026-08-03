#include <stdbool.h>
#include <float.h>
#include <math.h>

#include "pll.h"

#include "esp_log.h"

#define EPSILON 0.0001f

static const char * TAG = "pll";

esp_err_t pll_get_parameters(float target_freq, uint16_t fb_divider_min,
                             uint16_t fb_divider_max, uint8_t *fb_divider,
                             uint8_t *refdiv, uint8_t *postdiv1,
                             uint8_t *postdiv2, float *actual_freq)
{
    if (!isfinite(target_freq) || target_freq <= 0.0f ||
        fb_divider_min == 0 || fb_divider_min > fb_divider_max ||
        fb_divider_max > UINT8_MAX || fb_divider == NULL || refdiv == NULL ||
        postdiv1 == NULL || postdiv2 == NULL || actual_freq == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float best_freq = 0;
    uint8_t best_refdiv = 0, best_fb_divider = 0, best_postdiv1 = 0, best_postdiv2 = 0;
    float min_diff = FLT_MAX;
    float min_vco_freq = FLT_MAX;
    uint16_t min_postdiv = UINT16_MAX;

    for (uint8_t refdiv = 2; refdiv > 0; refdiv--) {
        for (uint8_t postdiv1 = 7; postdiv1 > 0; postdiv1--) {
            for (uint8_t postdiv2 = 7; postdiv2 > 0; postdiv2--) {
                uint16_t divider = refdiv * postdiv1 * postdiv2;
                long candidate = lroundf(target_freq / FREQ_MULT * divider);
                if (candidate < fb_divider_min || candidate > fb_divider_max ||
                    candidate > UINT8_MAX) {
                    continue;
                }
                uint16_t fb_divider = (uint16_t)candidate;
                if (postdiv1 > postdiv2 &&
                    fb_divider >= fb_divider_min && fb_divider <= fb_divider_max) {
                    float new_freq = FREQ_MULT * fb_divider / divider;
                    float curr_diff = fabs(target_freq - new_freq);
                    float vco_freq = FREQ_MULT * fb_divider / refdiv;
                    // Prioritize: 
                    // 1. Closest frequency to target
                    // 2. Lowest VCO frequency
                    // 3. Lowest postdiv1 * postdiv2
                    if (curr_diff < min_diff ||
                       (fabs(curr_diff - min_diff) < EPSILON && vco_freq < min_vco_freq) ||
                       (fabs(curr_diff - min_diff) < EPSILON && fabs(vco_freq - min_vco_freq) < EPSILON && postdiv1 * postdiv2 < min_postdiv)) {
                        min_diff = curr_diff;
                        min_vco_freq = vco_freq;
                        min_postdiv = postdiv1 * postdiv2;
                        best_freq = new_freq;
                        best_refdiv = refdiv;
                        best_fb_divider = fb_divider;
                        best_postdiv1 = postdiv1;
                        best_postdiv2 = postdiv2;
                    }
                }
            }
        }
    }

    if (best_fb_divider == 0 || best_refdiv == 0 || best_postdiv1 == 0 ||
        best_postdiv2 == 0 || !isfinite(best_freq) || best_freq <= 0.0f) {
        ESP_LOGE(TAG, "No valid PLL parameters for %g MHz", target_freq);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Frequency: %g MHz (fb_divider: %d, refdiv: %d, postdiv1: %d, postdiv2: %d)", best_freq, best_fb_divider, best_refdiv, best_postdiv1, best_postdiv2);

    *actual_freq = best_freq;
    *fb_divider = best_fb_divider;
    *refdiv = best_refdiv;
    *postdiv1 = best_postdiv1;
    *postdiv2 = best_postdiv2;
    return ESP_OK;
}
