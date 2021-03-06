﻿/*
	Satellite Attitude Dynamics Stepper

	@author		:	siddharth deore
	@licence	:	MIT
*/

#include "Satellite.h"
#include <iostream>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include "ode_wrapper.h"

#include <fstream>
#include <windows.h>
HANDLE hOut= GetStdHandle(STD_OUTPUT_HANDLE);;
void clearScreen()
{
	COORD Position;

	Position.X = 0;
	Position.Y = 0;
	SetConsoleCursorPosition(hOut, Position);
}

// set bool visible = 0 - invisible, bool visible = 1 - visible
void setcursor(bool visible, DWORD size) 
{
	if (size == 0)
	{
		// default cursor size Changing to numbers from 1 to 20, decreases cursor width
		size = 20;	
	}
	CONSOLE_CURSOR_INFO lpCursor;
	lpCursor.bVisible = visible;
	lpCursor.dwSize = size;
	SetConsoleCursorInfo(hOut, &lpCursor);
}
namespace odeint = boost::numeric::odeint;

// Static variables common to all instance
double Satellite::Ixx = 1.0;
double Satellite::Iyy = 1.0;
double Satellite::Izz = 1.0;

double Satellite::Kp = 0.0;
double Satellite::Kd = 1.0;

double Satellite::qd[4] = { 1.0,0.0,0.0,0.0 };


void Satellite::Satelliite() {
	state_type X = { { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  } }; // initial conditions
}

int Satellite::setQuaternion(double q0, double q1, double q2, double q3) {
	this->X[0] = q0;
	this->X[1] = q1;
	this->X[2] = q2;
	this->X[3] = q3;
	return 0;
}

int Satellite::setAngulerVeolcities(double wx, double wy, double wz) {
	this->X[4] = wx;
	this->X[5] = wy;
	this->X[6] = wz;
	return 0;
}

int Satellite::setState(double q0, double q1, double q2, double q3, double wx, double wy, double wz) {
	setQuaternion(q0, q1, q2, q3);
	setAngulerVeolcities(wx, wy, wz);
	return 0;
}

int Satellite::getState(double& q0, double& q1, double& q2, double& q3, double& wx, double& wy, double& wz) {
	q0 = this->X[0];
	q1 = this->X[1];
	q2 = this->X[2];
	q3 = this->X[3];
	wx = this->X[5];
	wy = this->X[6];
	wz = this->X[7];
	return 0;
}

void Satellite::dynamics(const state_type& x, state_type& dxdt, const double t) {
	double q0 = x[0];
	double q1 = x[1];
	double q2 = x[2];
	double q3 = x[3];
	double norm = normalizeQuaternions(q0, q1, q2, q3);
	dxdt[0] = 0.5 * (q1 * x[6] - q2 * x[5] + q3 * x[4]);
	dxdt[1] = 0.5 * (q2 * x[4] - q0 * x[6] + q3 * x[5]);
	dxdt[2] = 0.5 * (q0 * x[5] - q1 * x[4] + q3 * x[6]);
	dxdt[3] = 0.5 * (-q0 * x[4] - q1 * x[5] - q2 * x[6]);
	
	qe[0] = q0 * this->qd[3] + q1 * this->qd[2] - q2 * this->qd[1] - q3 * this->qd[0];
	qe[1] = q2 * this->qd[0] - q0 * this->qd[2] + q1 * this->qd[3] - q3 * this->qd[1];
	qe[2] = q0 * this->qd[1] - q1 * this->qd[0] + q2 * this->qd[3] - q3 * this->qd[2];
	qe[3] = q0 * this->qd[0] + q1 * this->qd[1] + q2 * this->qd[2] + q3 * this->qd[3];
	double PD[3] = {
					this.Kp * qe[0] * qe[3] + this.Kd * x[4],
					this.Kp * qe[1] * qe[3] + this.Kd * x[5],
					this.Kp * qe[2] * qe[3] + this.Kd * x[6]
				};

	dxdt[4] = ((this->Iyy - this->Izz) * x[5] * x[6] - PD[0]) / this->Ixx;
	dxdt[5] = ((this->Izz - this->Ixx) * x[6] * x[4] - PD[1]) / this->Iyy;
	dxdt[6] = ((this->Ixx - this->Iyy) * x[4] * x[5] - PD[2]) / this->Izz;
}

double Satellite::normalizeQuaternions(double& _q0, double& _q1, double& _q2, double& _q3)
{
	double norm = std::sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
	_q0 /= norm;
	_q1 /= norm;
	_q2 /= norm;
	_q3 /= norm;
	return norm;
}


int Satellite::step(double final_time, double dt, state_type& new_state)
{
	odeint::integrate(
		make_ode_wrapper(Satellite(), &Satellite::dynamics), // ODE funtion
		X,				// Initial state
		0.0,			// initial time
		final_time,		// final time
		dt,				// timestep
		make_observer_wrapper(Satellite(), &Satellite::write_state)// observer function
	);

	// return value to caller
	new_state = X;
	return 0;
}

void Satellite::setTargetQuaternion(double q0, double q1, double q2, double q3)
{
	this->qd[0] = q0;
	this->qd[1] = q1;
	this->qd[2] = q2;
	this->qd[3] = q3;
}

void Satellite::write_state(const state_type& state, const double t) {
	
	clearScreen(); // clear screen 
	setcursor(0, 1); // Hide cursor

	std::cout << char(218) << std::string(83, char(196))<< char(191) << std::endl;
	std::cout<<"|     time    |    q0   |    q1   |    q2   |    q3   |    wx   |    wy   |    wz   |" << std::endl;
	std::cout << char(195) << std::string(83, char(196)) << char(180) << std::endl;
	std::cout << "|" << std::setw(12) << std::fixed << std::setprecision(4) << t << " |";
	std::cout << std::setw(8) << state[0] << " |";
	std::cout << std::setw(8) << state[1] << " |";
	std::cout << std::setw(8) << state[2] << " |";
	std::cout << std::setw(8) << state[3] << " |";
	std::cout << std::setw(8) << state[4] << " |";
	std::cout << std::setw(8) << state[5] << " |";
	std::cout << std::setw(8) << state[6] << " |" << std::endl;
	std::cout << char(192) << std::string(83, char(196)) << char(217) << std::endl;
	
	//Progress bar
	std::cout << std::string(int(t / 10000 * 85), char(178)) << std::string(int(85 - t / 10000 * 85), char(176)) << std::endl;
	
	// write to file
	std::string s =
		std::to_string(t) +		   ", " +
		std::to_string(state[0]) + ", " +
		std::to_string(state[1]) + ", " +
		std::to_string(state[2]) + ", " +
		std::to_string(state[3]) + ", " +
		std::to_string(state[4]) + ", " +
		std::to_string(state[5]) + ", " +
		std::to_string(state[6]) + "\n";
	std::ofstream outfile;

	outfile.open("results.csv", std::ios_base::app); // append instead of overwrite
	outfile << s;
	outfile.close();
	//

}

void Satellite::setInnertia(double Ix, double Iy, double Iz) {
	this->Ixx = Ix;
	this->Iyy = Iy;
	this->Izz = Iz;
}

void Satellite::setControllerGains(double Kp, double Kd)
{
	this->Kp = Kp;
	this->Kd = Kd;
}
