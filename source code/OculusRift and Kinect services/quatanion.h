#ifndef _QUATERNION_H_
#define _QUATERNION_H_

//#include <math.h>

void Matrix2Quaternion(float m[3][3], float q[4])
{
	double w = sqrt(1.0 + m[0][0] + m[1][1] + m[2][2]) / 2.0;
	double w4 = 4.0 * w;
	double x = (m[2][1] - m[1][2]) / w4;
	double y = (m[0][2] - m[2][0]) / w4;
	double z = (m[1][0] - m[0][1]) / w4;

	q[0] = w;
	q[1] = x;
	q[2] = y;
	q[3] = z;
}

void Quaternion2Matrix(float q[4], float m[3][3])
{
	double sqw = q[0]*q[0];
	double sqx = q[1]*q[1];
	double sqy = q[2]*q[2];
	double sqz = q[3]*q[3];

	m[0][0] =  sqx - sqy - sqz + sqw;	// since sqw + sqx + sqy + sqz =1
	m[1][1] = -sqx + sqy - sqz + sqw;
	m[2][2] = -sqx - sqy + sqz + sqw;

	double tmp1 = q[1]*q[2];
	double tmp2 = q[3]*q[0];

	m[1][0] = 2.0 * (tmp1 + tmp2);
	m[0][1] = 2.0 * (tmp1 - tmp2);

	tmp1 = q[1]*q[3];
	tmp2 = q[2]*q[0];

	m[2][0] = 2.0 * (tmp1 - tmp2);
	m[0][2] = 2.0 * (tmp1 + tmp2);

	tmp1 = q[2]*q[3];
	tmp2 = q[1]*q[0];

	m[2][1] = 2.0 * (tmp1 + tmp2);
	m[1][2] = 2.0 * (tmp1 - tmp2);
}

// x����]
void get_Rx(float q[4], float *rx){
	*rx = atan2( 2*(q[2]*q[3] + q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
}

// y����]
void get_Ry(float q[4], float *ry){
	*ry = asin( -2*(q[1]*q[3] - q[0]*q[2]));
}

// z����]
void get_Rz(float q[4], float *rz){
	*rz = atan2( 2*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
}

void get_rotation( float q[4], double *rx, double *ry, double *rz )
{
	*rx = atan2( 2*(q[2]*q[3] + q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
	*ry = asin( -2*(q[1]*q[3] - q[0]*q[2]));
	*rz = atan2( 2*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
}

#endif
