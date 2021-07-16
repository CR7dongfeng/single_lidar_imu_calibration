#include <iostream>
#include <string>

void utc_to_gps(const double utc_sec, unsigned int& gps_week, double& gps_sec) {
	double temp_sec = utc_sec + 18 - 315936000;
	gps_week = temp_sec / (7*24*3600);
	gps_sec = temp_sec - gps_week * 7*24*3600;
}

void gps_to_utc(double& utc_sec, const unsigned int gps_week, const double gps_sec) {
	utc_sec = 315936000 + gps_week*7*24*3600 + gps_sec - 18;
}


int main(int argc, char** argv) {
	unsigned int gps_week = 0;
	double gps_sec = 0.0;
	double utc_sec = 0.0;
	std::cout.precision(25);
	
	if( argc == 2 ) {
		utc_sec = std::stod(std::string(argv[1]));
		utc_to_gps(utc_sec, gps_week, gps_sec);
		std::cout << "utc to gps ： " << std::endl;
		std::cout << "utc_sec : " << utc_sec << std::endl;
		std::cout << "gps_week : " << gps_week << std::endl;
		std::cout << "gps_sec : " << gps_sec << std::endl;
	}
	else if( argc == 3 ) {
		gps_week = std::stoi(std::string(argv[1]));
		gps_sec = std::stod(std::string(argv[2]));
		gps_to_utc(utc_sec, gps_week, gps_sec);
		std::cout << "gps to utc ： " << std::endl;
		std::cout << "utc_sec : " << utc_sec << std::endl;
		std::cout << "gps_week : " << gps_week << std::endl;
		std::cout << "gps_sec : " << gps_sec << std::endl;
	}
	else {
		std::cout << "parameter error!" << std::endl;
		return -1;
	}
	
	return 0;
}

