/*
 * DCM.c
 *
 *  Created on: Sep 17, 2026
 *      Author: Admin
 */

#include "DCM.h"
#include <math.h>

void computeAngles()
{
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * R2D;
	float sinp = -2.0f * (q1*q3 - q0*q2);
	if (sinp > 1.0f) sinp = 1.0f;
	if (sinp < -1.0f) sinp = -1.0f;
	pitch = asinf(sinp) * R2D;
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * R2D;
}

