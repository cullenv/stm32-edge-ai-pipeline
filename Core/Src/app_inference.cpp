/*
 * app_inference.cpp
 *
 *  Created on: Mar 26, 2026
 *      Author: culle
 */


#include "app_inference.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include <stdio.h>

void run_ml_inference(float rms_x, float rms_y, float rms_z) {

    // 1. Pack your 3 C-variables into a C++ array
    float features[3] = { rms_x, rms_y, rms_z };

    // 2. Wrap the array in Edge Impulse's custom signal format
    signal_t signal;
    int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        printf("Failed to create AI signal (%d)\r\n", err);
        return;
    }

    // 3. Run the Neural Network!
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

    if (res != EI_IMPULSE_OK) {
        printf("Failed to run AI classifier (%d)\r\n", res);
        return;
    }

    // 4. Print the AI's physical diagnosis
    printf("--- AI DIAGNOSIS ---\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        printf("  %s: %.2f%%\r\n", result.classification[i].label, result.classification[i].value * 100.0f);
    }
    printf("--------------------\r\n\n");
}

