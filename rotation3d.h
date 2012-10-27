/*
 c-rotate3d - Helper for 3D rotation around axis or with quaternion.
 BSD license.
 by Sven Nilsen, 2012
 http://www.cutoutpro.com
 
 Version: 0.000 in angular degrees version notation
 http://isprogrammingeasy.blogspot.no/2012/08/angular-degrees-versioning-notation.html
 */
/*
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 The views and conclusions contained in the software and documentation are those
 of the authors and should not be interpreted as representing official policies,
 either expressed or implied, of the FreeBSD Project.
 */

/*
 
 A helper for rotating around a center in 3D.
 1. Set the rotation center.
 2. Set the type of rotation.
 3.1 Rotate a single point.
 3.2 Rotate array of points.
 3.3 Generate translaterotate matrix.
 
 */

// Helper data for both axis, angle rotation and quaternion.
typedef struct
{
	double px, py, pz;
	int rotation_type;
	double ax, ay, az, cos, sin;
	double qw, qx, qy, qz;
} rotation3d_helper;

// Sets the rotation center.
void rotation3d_Step1_SetRotationCenter
(rotation3d_helper *helper, double x, double y, double z);

// Sets axis (unit vector) and angle to rotate around.
void rotation3d_Step2_SetAxisAndAngle
(rotation3d_helper *helper, double x, double y, double z, double angle);

// Sets rotation by quaternion.
void rotation3d_Step2_SetQuaternion
(rotation3d_helper *helper, double q[]);

// Sets quaternion with single precison.
void rotation3d_Step2_SetQuaternionf
(rotation3d_helper *helper, float q[]);

// Performs the rotation and stores result in same place.
void rotation3d_Step3_RotatePoint
(const rotation3d_helper *helper, double *x, double *y, double *z);

// Performs rotations on n points.
// Use the step argument 3 if the points are packed xyz.
// Matrix is generated from quaternion to save operations.
void rotation3d_Step3_RotatePoints
(const rotation3d_helper *helper, int n, int step,
 double *x, double *y, double *z);

// Performs rotations on n single precision floating points.
void rotation3d_Step3_RotatePointsf
(const rotation3d_helper *helper, int n, int step,
 float *x, float *y, float *z);

// Performs rotation and stores result with single precision.
void rotation3d_Step3_RotatePointf
(const rotation3d_helper *helper, float *x, float *y, float *z);

// Stores rotation around center as a 4x4 matrix.
// Points need to subtract rotation center before transforming.
//  0,  1,  2,  3
//  4,  5,  6,  7
//  8,  9, 10, 11
// 12, 13, 14, 15
void rotation3d_Step3_ToTranslateRotateMatrix
(const rotation3d_helper *helper, double *mat);

// Generates single precision floating point matrix.
void rotation3d_Step3_ToTranslateRotateMatrixf
(const rotation3d_helper *helper, float *mat);

