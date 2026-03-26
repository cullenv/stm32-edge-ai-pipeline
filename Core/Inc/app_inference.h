/*
 * app_inference.h
 *
 *  Created on: Mar 26, 2026
 *      Author: culle
 */

#ifndef INC_APP_INFERENCE_H_
#define INC_APP_INFERENCE_H_

#ifdef __cplusplus //lets C to to its C++ brother
extern "C" {
#endif

// The function our main.c loop will call
void run_ml_inference(float rms_x, float rms_y, float rms_z);

#ifdef __cplusplus
}
#endif

#endif /* INC_APP_INFERENCE_H_ */
