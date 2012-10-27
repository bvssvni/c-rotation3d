
#include <math.h>

#include "rotation3d.h"

#define ROTATION_AXIS_ANGLE 1
#define ROTATION_QUATERNION 2

void rotation3d_Step1_SetRotationCenter
(rotation3d_helper *helper, double x, double y, double z)
{
	helper->px = x; helper->py = y; helper->pz = z;
}

void rotation3d_Step2_SetAxisAndAngle
(rotation3d_helper *helper, double x, double y, double z, double angle)
{
	helper->ax = x; helper->ay = y; helper->az = z;
	helper->cos = cos(angle);
	helper->sin = sin(angle);
	helper->rotation_type = ROTATION_AXIS_ANGLE;
}

void rotation3d_Step2_SetQuaternion
(rotation3d_helper *helper, double q[])
{
	helper->qw = q[0]; helper->qx = q[1]; helper->qy = q[2]; helper->qz = q[3];
	helper->rotation_type = ROTATION_QUATERNION;
}

void rotation3d_Step2_SetQuaternionf
(rotation3d_helper *helper, float q[])
{
	helper->qw = q[0]; helper->qx = q[1]; helper->qy = q[2]; helper->qz = q[3];
	helper->rotation_type = ROTATION_QUATERNION;
}

void rotation3d_Step3_RotatePoint
(const rotation3d_helper *helper, double *x, double *y, double *z)
{
	double px = *x - helper->px;
	double py = *y - helper->py;
	double pz = *z - helper->pz;
	
	if (helper->rotation_type == ROTATION_AXIS_ANGLE) {
		// Rotate point using axis and angle.
		double c = helper->cos;
		double mc = 1-c;
		double s = helper->sin;
		double ax = helper->ax;
		double ay = helper->ay;
		double az = helper->az;
	
		*x = px*(c+ax*ax*mc) + py*(ax*ay*mc-az*s) + pz*(ax*az*mc+ay*s);
		*y = px*(ay*ax*mc+az*s) + py*(c+ay*ay*mc) + pz*(ay*az*mc-ax*s);
		*z = px*(az*ax*mc-ay*s) + py*(az*ay*mc+ax*s) + pz*(c+az*az*mc);
	} else {
		// Rotate point using quaternion.
		double qw = helper->qw;
		double qx = helper->qx;
		double qy = helper->qy;
		double qz = helper->qz;
		
		// a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
		double qpw = - qx * px - qy * py - qz * pz;
		// a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
		double qpx = qw * px + qy * pz - qz * py;
		// a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
		double qpy = qw * py - qx * pz + qz * px;
		// a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2
		double qpz = qw * pz + qx * py - qy * px;
		
		//   a1  *  b2 + b1  *  a2 + c1  *  d2 - d1  *  c2
		*x = qpw * -qx + qpx *  qw + qpy * -qz - qpz * -qy;
		//   a1  *  c2 - b1  *  d2 + c1  *  a2 + d1  *  b2
		*y = qpw * -qy - qpx * -qz + qpy *  qw + qpz * -qx;
		//   a1  *  d2 + b1  *  c2 - c1  *  b2 + d1  *  a2
		*z = qpw * -qz + qpx * -qy - qpy * -qx + qpz *  qw;
	}
	
	*x += helper->px;
	*y += helper->py;
	*z += helper->pz;
}

void rotation3d_Step3_RotatePoints
(const rotation3d_helper *helper, int n, int step,
 double *x, double *y, double *z)
{
	double px, py, pz;
	double hpx = helper->px;
	double hpy = helper->py;
	double hpz = helper->pz;
	
	if (helper->rotation_type == ROTATION_AXIS_ANGLE) {
		// Rotate point using axis and angle.
		double c = helper->cos;
		double mc = 1-c;
		double s = helper->sin;
		double ax = helper->ax;
		double ay = helper->ay;
		double az = helper->az;
		double m[] = {
			c+ax*ax*mc, ax*ay*mc-az*s, ax*az*mc+ay*s,
			ay*ax*mc+az*s, c+ay*ay*mc, ay*az*mc-ax*s,
			az*ax*mc-ay*s, az*ay*mc+ax*s, c+az*az*mc
		};
		
		int i;
		int len = n * step;
		for (i = 0; i < len; i += step) {
			px = x[i] - hpx; py = y[i] - hpy; pz = z[i] - hpz;
			x[i] = hpx + px*m[0] + py*m[1] + pz*m[2];
			y[i] = hpy + px*m[3] + py*m[4] + pz*m[5];
			z[i] = hpz + px*m[6] + py*m[7] + pz*m[8];
		}
	} else {
		// Generate matrix to save operations.
		double mat[16];
		rotation3d_Step3_ToTranslateRotateMatrix(helper, mat);
		
		double px, py, pz;
		int i;
		int len = n * step;
		for (i = 0; i < len; i += step) {
			px = x[i] - hpx; py = y[i] - hpy; pz = z[i] - hpz;
			x[i] = hpx + px*mat[0] + py*mat[1] + pz*mat[2];
			y[i] = hpy + px*mat[4] + py*mat[5] + pz*mat[6];
			z[i] = hpz + px*mat[8] + py*mat[9] + pz*mat[10];
		}
	}
}

void rotation3d_Step3_RotatePointf
(const rotation3d_helper *helper, float *x, float *y, float *z)
{
	double px = *x - helper->px;
	double py = *y - helper->py;
	double pz = *z - helper->pz;
	
	if (helper->rotation_type == ROTATION_AXIS_ANGLE) {
		// Rotate point using axis and angle.
		double c = helper->cos;
		double mc = 1-c;
		double s = helper->sin;
		double ax = helper->ax;
		double ay = helper->ay;
		double az = helper->az;
		
		*x = px*(c+ax*ax*mc) + py*(ax*ay*mc-az*s) + pz*(ax*az*mc+ay*s);
		*y = px*(ay*ax*mc+az*s) + py*(c+ay*ay*mc) + pz*(ay*az*mc-ax*s);
		*z = px*(az*ax*mc-ay*s) + py*(az*ay*mc+ax*s) + pz*(c+az*az*mc);
	} else {
		// Rotate point using quaternion.
		double qw = helper->qw;
		double qx = helper->qx;
		double qy = helper->qy;
		double qz = helper->qz;
		
		// a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
		double qpw = - qx * px - qy * py - qz * pz;
		// a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
		double qpx = qw * px + qy * pz - qz * py;
		// a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
		double qpy = qw * py - qx * pz + qz * px;
		// a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2
		double qpz = qw * pz + qx * py - qy * px;
		
		//   a1  *  b2 + b1  *  a2 + c1  *  d2 - d1  *  c2
		*x = qpw * -qx + qpx *  qw + qpy * -qz - qpz * -qy;
		//   a1  *  c2 - b1  *  d2 + c1  *  a2 + d1  *  b2
		*y = qpw * -qy - qpx * -qz + qpy *  qw + qpz * -qx;
		//   a1  *  d2 + b1  *  c2 - c1  *  b2 + d1  *  a2
		*z = qpw * -qz + qpx * -qy - qpy * -qx + qpz *  qw;
	}
	
	*x += helper->px;
	*y += helper->py;
	*z += helper->pz;
}

void rotation3d_Step3_RotatePointsf
(const rotation3d_helper *helper, int n, int step,
 float *x, float *y, float *z)
{
	double px, py, pz;
	double hpx = helper->px;
	double hpy = helper->py;
	double hpz = helper->pz;
	
	if (helper->rotation_type == ROTATION_AXIS_ANGLE) {
		// Rotate point using axis and angle.
		double c = helper->cos;
		double mc = 1-c;
		double s = helper->sin;
		double ax = helper->ax;
		double ay = helper->ay;
		double az = helper->az;
		double m[] = {
			c+ax*ax*mc, ax*ay*mc-az*s, ax*az*mc+ay*s,
			ay*ax*mc+az*s, c+ay*ay*mc, ay*az*mc-ax*s,
			az*ax*mc-ay*s, az*ay*mc+ax*s, c+az*az*mc
		};
		
		int i;
		int len = n * step;
		for (i = 0; i < len; i += step) {
			px = x[i] - hpx; py = y[i] - hpy; pz = z[i] - hpz;
			x[i] = hpx + px*m[0] + py*m[1] + pz*m[2];
			y[i] = hpy + px*m[3] + py*m[4] + pz*m[5];
			z[i] = hpz + px*m[6] + py*m[7] + pz*m[8];
		}
	} else {
		// Generate matrix to save operations.
		double mat[16];
		rotation3d_Step3_ToTranslateRotateMatrix(helper, mat);
		
		double px, py, pz;
		int i;
		int len = n * step;
		for (i = 0; i < len; i += step) {
			px = x[i] - hpx; py = y[i] - hpy; pz = z[i] - hpz;
			x[i] = hpx + px*mat[0] + py*mat[1] + pz*mat[2];
			y[i] = hpy + px*mat[4] + py*mat[5] + pz*mat[6];
			z[i] = hpz + px*mat[8] + py*mat[9] + pz*mat[10];
		}
	}
}

void rotation3d_Step3_ToTranslateRotateMatrix
(const rotation3d_helper *helper, double mat[])
{
	if (helper->rotation_type == ROTATION_AXIS_ANGLE) {
		double c = helper->cos;
		double mc = 1-c;
		double s = helper->sin;
		double ax = helper->ax;
		double ay = helper->ay;
		double az = helper->az;
		
		// 1st row.
		mat[0] = c+ax*ax*mc;
		mat[1] = ax*ay*mc-az*s;
		mat[2] = ax*az*mc+ay*s;
		mat[3] = helper->px;
		
		// 2nd row.
		mat[4] = ay*ax*mc+az*s;
		mat[5] = c+ay*ay*mc;
		mat[6] = ay*az*mc-ax*s;
		mat[7] = helper->py;
		
		// 3rd row.
		mat[8] = az*ax*mc-ay*s;
		mat[9] = az*ay*mc+ax*s;
		mat[10] = c+az*az*mc;
		mat[11] = helper->pz;
		
		// 4th row.
		mat[12] = 0;
		mat[13] = 0;
		mat[14] = 0;
		mat[15] = 1;
	} else {
		double a = helper->qw;
		double b = helper->qx;
		double c = helper->qy;
		double d = helper->qz;
		
		// 1st row.
		mat[0] = a*a + b*b - c*c - d*d;
		mat[1] = 2*b*c - 2*a*d;
		mat[2] = 2*b*d + 2*a*c;
		mat[3] = helper->px;
		
		// 2nd row.
		mat[4] = 2*b*c + 2*a*d;
		mat[5] = a*a - b*b + c*c - d*d;
		mat[6] = 2*c*d - 2*a*b;
		mat[7] = helper->py;
		
		// 3rd row.
		mat[8] = 2*b*d - 2*a*c;
		mat[9] = 2*c*d + 2*a*b;
		mat[10] = a*a - b*b - c*c + d*d;
		mat[11] = helper->pz;
		
		// 4th row.
		mat[12] = 0;
		mat[13] = 0;
		mat[14] = 0;
		mat[15] = 1;
	}
}

void rotation3d_Step3_ToTranslateRotateMatrixf
(const rotation3d_helper *helper, float mat[])
{
	if (helper->rotation_type == ROTATION_AXIS_ANGLE) {
		double c = helper->cos;
		double mc = 1-c;
		double s = helper->sin;
		double ax = helper->ax;
		double ay = helper->ay;
		double az = helper->az;
		
		// 1st row.
		mat[0] = c+ax*ax*mc;
		mat[1] = ax*ay*mc-az*s;
		mat[2] = ax*az*mc+ay*s;
		mat[3] = helper->px;
		
		// 2nd row.
		mat[4] = ay*ax*mc+az*s;
		mat[5] = c+ay*ay*mc;
		mat[6] = ay*az*mc-ax*s;
		mat[7] = helper->py;
		
		// 3rd row.
		mat[8] = az*ax*mc-ay*s;
		mat[9] = az*ay*mc+ax*s;
		mat[10] = c+az*az*mc;
		mat[11] = helper->pz;
		
		// 4th row.
		mat[12] = 0;
		mat[13] = 0;
		mat[14] = 0;
		mat[15] = 1;
	} else {
		double a = helper->qw;
		double b = helper->qx;
		double c = helper->qy;
		double d = helper->qz;
		
		// 1st row.
		mat[0] = a*a + b*b - c*c - d*d;
		mat[1] = 2*b*c - 2*a*d;
		mat[2] = 2*b*d + 2*a*c;
		mat[3] = helper->px;
		
		// 2nd row.
		mat[4] = 2*b*c + 2*a*d;
		mat[5] = a*a - b*b + c*c - d*d;
		mat[6] = 2*c*d - 2*a*b;
		mat[7] = helper->py;
		
		// 3rd row.
		mat[8] = 2*b*d - 2*a*c;
		mat[9] = 2*c*d + 2*a*b;
		mat[10] = a*a - b*b - c*c + d*d;
		mat[11] = helper->pz;
		
		// 4th row.
		mat[12] = 0;
		mat[13] = 0;
		mat[14] = 0;
		mat[15] = 1;
	}
}

