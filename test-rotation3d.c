
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "rotation3d.h"

#define PI 3.141592653589793
#define EPS 0.000000000001

// Converts axis and rotation angle {x, y, z, angle}
// into a quaternion {w, x, y, z}.
void axisAngleToQuaternion(double *ax, double *q)
{
	double half_angle = ax[3]/2;
	double half_cos = cos(half_angle);
	double half_sin = sin(half_angle);
	q[0] = half_cos;
	q[1] = ax[0] * half_sin;
	q[2] = ax[1] * half_sin;
	q[3] = ax[2] * half_sin;
}

void test(void)
{
	{
		rotation3d_helper helper = {};
		rotation3d_Step1_SetRotationCenter(&helper, 0, 0, 0);
		rotation3d_Step2_SetAxisAndAngle(&helper, 0, 1, 0, PI);
		double x = 1, y = 0, z = 0;
		rotation3d_Step3_RotatePoint(&helper, &x, &y, &z);
		// printf("x %g y %g z %g\n", x, y, z);
		assert(fabs(x - -1) < EPS);
		assert(fabs(y - 0) < EPS);
		assert(fabs(z - 0) < EPS);
	}
	{
		rotation3d_helper helper = {};
		rotation3d_Step1_SetRotationCenter(&helper, 0, 0, 0);
		rotation3d_Step2_SetAxisAndAngle(&helper, 0, 1, 0, PI);
		double x = 0, y = 0, z = 1;
		rotation3d_Step3_RotatePoint(&helper, &x, &y, &z);
		// printf("x %g y %g z %g\n", x, y, z);
		assert(fabs(x - 0) < EPS);
		assert(fabs(y - 0) < EPS);
		assert(fabs(z - -1) < EPS);
	}
	{
		rotation3d_helper helper = {};
		rotation3d_Step1_SetRotationCenter(&helper, 0, 0, 0);
		rotation3d_Step2_SetAxisAndAngle(&helper, 0, 0, 1, PI);
		double mat[16];
		rotation3d_Step3_ToTranslateRotateMatrix(&helper, mat);
		// printf("%g\n", mat[0]);
		// printf("%g\n", mat[1]);
		// printf("%g\n", mat[4]);
		// printf("%g\n", mat[5]);
		assert(fabs(mat[0] - -1) < EPS);
		assert(fabs(mat[1] - 0) < EPS);
		assert(fabs(mat[4] - 0) < EPS);
		assert(fabs(mat[5] - -1) < EPS);
	}
	{
		// Quaternion, single point rotation.
		rotation3d_helper helper = {};
		double q[4];
		double ax[] = {0, 1, 0, PI};
		axisAngleToQuaternion(ax, q);
		rotation3d_Step1_SetRotationCenter(&helper, 0, 0, 0);
		rotation3d_Step2_SetQuaternion(&helper, q);
		double x = 1, y = 0, z = 0;
		rotation3d_Step3_RotatePoint(&helper, &x, &y, &z);
		// printf("x %g y %g z %g\n", x, y, z);
		assert(fabs(x - -1) < EPS);
		assert(fabs(y - 0) < EPS);
		assert(fabs(z - 0) < EPS);
	}
	{
		// Quaternion, to translaterotate matrix.
		rotation3d_helper helper = {};
		double q[4];
		double ax[] = {0, 0, 1, PI};
		axisAngleToQuaternion(ax, q);
		rotation3d_Step1_SetRotationCenter(&helper, 0, 0, 0);
		rotation3d_Step2_SetQuaternion(&helper, q);
		double mat[16];
		rotation3d_Step3_ToTranslateRotateMatrix(&helper, mat);
		// printf("%g\n", mat[0]);
		// printf("%g\n", mat[1]);
		// printf("%g\n", mat[4]);
		// printf("%g\n", mat[5]);
		assert(fabs(mat[0] - -1) < EPS);
		assert(fabs(mat[1] - 0) < EPS);
		assert(fabs(mat[4] - 0) < EPS);
		assert(fabs(mat[5] - -1) < EPS);
	}
	{
		// Quaternion, fake array rotation.
		rotation3d_helper helper = {};
		double q[4];
		double ax[] = {0, 1, 0, PI};
		axisAngleToQuaternion(ax, q);
		rotation3d_Step1_SetRotationCenter(&helper, 0, 0, 0);
		rotation3d_Step2_SetQuaternion(&helper, q);
		double x = 1, y = 0, z = 0;
		rotation3d_Step3_RotatePoints(&helper, 1, 1, &x, &y, &z);
		// printf("x %g y %g z %g\n", x, y, z);
		assert(fabs(x - -1) < EPS);
		assert(fabs(y - 0) < EPS);
		assert(fabs(z - 0) < EPS);
	}
}

int main(int argc, char *argv[])
{
	test();
	printf("rotation3d unit tests succeeded!\n");
	return 0;
}
