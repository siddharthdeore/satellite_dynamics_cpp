/*
	Satellite Attitude Dynamics Stepper

	@author		:	siddharth deore
	@licence	:	MIT
*/

#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
// reading a text file
#include <iostream>
#include <fstream>
#include <string>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include "Satellite.h"
#include <ctime>
using namespace std;
using namespace boost::numeric::odeint;


int main(int argc, char** argv)
{
	system("cls");
	// Initial condition
	double IC[7] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
	// Read Configration file for initial condiion Each line is state, TODO:: some kind of parser!
	string line;
	ifstream myfile("conf.txt");
	int i = 0;
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			IC[i++] = std::stod(line);
			//cout << IC[i-1] <<std::endl;
		}
		myfile.close();
	}

	else cout << "Unable to open file";
	// Clean Results file

	std::ofstream outfile;

	outfile.open("results.csv"); // overwrite
	outfile << "time, q0, q1, q2, q3, wx, wy, wz" << endl;
	outfile.close();

	Satellite sat;
	Satellite::state_type x;
	time_t t1, t2;
	//double q0, q1, q2, q3, w0, w1, w2, w3;
	sat.setInnertia(1.0, 2.0, 3.0);
	sat.setState(IC[0], IC[1], IC[2], IC[3], IC[4], IC[5], IC[6]);
	time(&t1);
	sat.step(10000.0, 0.01, x);
	time(&t2);
	std::cout << "Execution time " << t2 - t1 << std::endl;
}
