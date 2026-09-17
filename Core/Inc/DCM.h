/*
 * DCM.h
 *
 *  Created on: Sep 17, 2026
 *      Author: Admin
 */

#ifndef INC_DCM_H_
#define INC_DCM_H_

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.141592
#define R2D 180.00f/3.141592f


extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern float roll, pitch, yaw;

void computeAngles(void);

#ifdef __cplusplus
}
#endif



#endif /* INC_DCM_H_ */
