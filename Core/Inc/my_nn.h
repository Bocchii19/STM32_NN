#ifndef MY_NN_H
#define MY_NN_H

#include <stdint.h>

// Hàm kích hoạt
float relu(float x);
float sigmoid(float x);
void softmax(const float* inputs, int n_inputs, float* outputs);

// Hàm tính toán forward cho một lớp
void forward_layer(const float* inputs, int n_inputs,
                   const float* weights, int n_neurons,
                   const float* biases, float* outputs,
                   float (*activation)(float));

// Hàm forward của toàn bộ mạng neural
void forward_nn(const float* input, float* output);

#endif // MY_NN_H
