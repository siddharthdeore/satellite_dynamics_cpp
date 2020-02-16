/*
	Satellite Attitude Dynamics Stepper

	@author		:	siddharth deore
	@licence	:	MIT
*/

#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

namespace odeint = boost::numeric::odeint;
typedef boost::array< double, 3 > state_type;

struct Satellite
{
	typedef boost::array< double, 7 > state_type;
private:
	state_type X;

public:
	static double Ixx, Iyy, Izz;
	void Satelliite();

	int setQuaternion(double q0, double q1, double q2, double q3);
	int setAngulerVeolcities(double wx, double wy, double wz);
	int setState(double q0, double q1, double q2, double q3, double wx, double wy, double wz);

	int getState(double& q0, double& q1, double& q2, double& q3, double& wx, double& wy, double& wz);

	void dynamics(const state_type& x, state_type& dxdt, const double t);
	double normalizeQuaternions(double& _q0, double& _q1, double& _q2, double& _q3);
	int step(double t, double dt, state_type& new_state);
	void setInnertia(double Ix, double Iy, double Iz);
	void write_state(const state_type& state, const double t);

};