/*
	Satellite Attitude Dynamics Stepper

	@author		:	siddharth deore
	@licence	:	MIT
*/

#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <boost/array.hpp>

#include <boost/numeric/odeint.hpp>
#include "Satellite.h"
#include <ctime>
using namespace std;
using namespace boost::numeric::odeint;


int main(int argc, char** argv)
{
	Satellite sat;
	Satellite::state_type x;
	time_t t1, t2;
	double q0, q1, q2, q3, w0, w1, w2, w3;
	sat.setInnertia(1.0, 2.0, 3.0);
	sat.setState(0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.001);
	time(&t1);
	sat.step(10000.0, 0.01, x);
	time(&t2);
	std::cout << "Execution time " << t2 - t1 << std::endl;
}
