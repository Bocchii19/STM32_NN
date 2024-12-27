/*
 * my_nn.c
 *
 *  Created on: Dec 26, 2024
 *      Author: Admin
 */

#include "my_nn.h"
#include "parameters.h"  // Đảm bảo tệp parameters.h được bao gồm
#include <math.h>

// Logic forward_nn
void forward_nn(const float* input, float* output) {
    float layer1_output[24];
    float layer2_output[24];
    float layer3_output[24];
    float layer4_output[16];
    float final_output[3];

    forward_layer(input, 3, layer1_weights, 24, layer1_biases, layer1_output, relu);
    forward_layer(layer1_output, 24, layer2_weights, 24, layer2_biases, layer2_output, sigmoid);
    forward_layer(layer2_output, 24, layer3_weights, 24, layer3_biases, layer3_output, sigmoid);
    forward_layer(layer3_output, 24, layer4_weights, 16, layer4_biases, layer4_output, sigmoid);
    forward_layer(layer4_output, 16, output_weights, 3, output_biases, final_output, NULL);
    softmax(final_output, 3, output);
}

float relu(float x) {
    return (x > 0) ? x : 0;
}
float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}
void softmax(const float* inputs, int n_inputs, float* outputs) {
    float max_input = inputs[0];
    for (int i = 1; i < n_inputs; i++) {
        if (inputs[i] > max_input) {
            max_input = inputs[i];
        }
    }

    float sum_exp = 0.0f;
    for (int i = 0; i < n_inputs; i++) {
        outputs[i] = expf(inputs[i] - max_input);
        sum_exp += outputs[i];
    }

    for (int i = 0; i < n_inputs; i++) {
        outputs[i] /= sum_exp;
    }
}
void forward_layer(const float* inputs, int n_inputs,
                   const float* weights, int n_neurons,
                   const float* biases, float* outputs,
                   float (*activation)(float)) {
    for (int i = 0; i < n_neurons; i++) {
        outputs[i] = biases[i];
        for (int j = 0; j < n_inputs; j++) {
            outputs[i] += inputs[j] * weights[i * n_inputs + j];
        }
        if (activation != NULL) {
            outputs[i] = activation(outputs[i]);
        }
    }
}





