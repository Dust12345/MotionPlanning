// TXTY_3CylinderObstaclesNavigation.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>

#include "Point.h"
#include "Potential.h"

using namespace std;

void write_program_file(vector<Point>); // Ausgabe einer EASYROB *.prg Datei zur Simulation der Ergebnisse und Verifikation
bool check_local_minimum(vector<Point>, Point);

/*****************************************************************************************/
int main(void)
{
	const double ds = 0.01f;
	const int nHind = 4;    // Anzahl der Hindernisse
	const int nRob = 1;     // Anzahl der Roboterglieder
	vector<Point> path;     // Create a vector containing integers
	Cylinder aHindernis[nHind];  // Unsere Hindernisse
	Cylinder Roboter[nRob];      // Roboter

	Potential pot1("Potential1");

	// Hindernisse initialisieren
	// Hierbei wird für jedes Hinderniss die Größe ( Skalierung ) und die Position im Raum gesetzt
	aHindernis[0].SetCenter(0., 0., 0.);  // outer bound
	aHindernis[0].SetRadius(1.);

	aHindernis[1].SetCenter(0.1, 0.1, 0.0);
	aHindernis[1].SetRadius(0.1);

	aHindernis[2].SetCenter(0.5, 0.1, 0.0);
	aHindernis[2].SetRadius(0.1);

	aHindernis[3].SetCenter(0.3, 0.34, 0.0);
	aHindernis[3].SetRadius(0.1);

	// Roboter initialisieren
	Roboter[0].SetCenter(0., 0., 0.0);
	Roboter[0].SetRadius(0.05);

	// Startzeit
	DWORD dwStart = GetTickCount();

	// Initialize start, goal, actPoint and heading
	//pot1.setStartPosition(0.3, -0.3); // local minimum
	//pot1.setStartPosition(0.2, -0.2); // local minimum
	pot1.setStartPosition(-0.1, -0.2); // no local minimum

	pot1.setGoalPosition(0.1, 0.4); // EASYROB

	Roboter[0].SetCenter(pot1.getStartPosition());

	pot1.setActPoint(pot1.getStartPosition());

	path.push_back(pot1.getStartPosition());

	Point robPos;
	bool goal_reached = false;
	bool local_minimum_reached = false;

	while (!goal_reached && !local_minimum_reached)
	{
		goal_reached = pot1.update_cylinder(aHindernis, Roboter, nHind);
		robPos = pot1.getRobPos();
		local_minimum_reached = check_local_minimum(path, robPos);
		path.push_back(pot1.getRobPos()); // speichern des Aktuellen Punktes in vector<Point> path
		cout << robPos.x << " " << robPos.y << endl; // Ausgabe auf Konsole
	}

	// Zeit für das Aufstellen des Konfigurationsraumes ausgeben ( in ms )
	DWORD dwElapsed = GetTickCount() - dwStart;
	if (local_minimum_reached)
		cout << "local minimum is reached" << endl;
	if (goal_reached)
		cout << "goal is reached" << endl;
	write_program_file(path);
	printf("\nBerechnung dauerte %d ms\n", dwElapsed);


	return 0;
}

/****************************************************************************************/
// Chekcs, if local minimum is reached
bool check_local_minimum(vector<Point> path, Point act)
{
	//simple check if the current position was reached bevore

	double errorMargin = 0.000005;

	for (int i = 0; i < path.size() - 1; i++)
	{

		double xDiff = abs(act.x - path[i].x);
		double yDiff = abs(act.y - path[i].y);

		if (xDiff <= errorMargin && yDiff <= errorMargin) {
			return true;
		}
	}
	return false;
}

/*****************************************************************************************/
// Ausgabe einer EASYROB *.prg Datei zur Simulation der Ergebnisse und Verifikation
void write_program_file(vector<Point> path)
{
	ofstream myfile;
	myfile.open("3CylinderObstaclesPotential.prg");
	// Iterate and print values of path
	vector<Point>::const_iterator i;
	myfile << "ProgramFile" << endl;
	//<< "EndInit" << endl << "SPEED_PTP_OV   80.0000" << endl << "ACCEL_PTP_OV   100.0000" << endl
	//<< "OV_PRO         100.0000" << endl;
	myfile << "JUMP_TO_AX  " << path.begin()->x << "  " << path.begin()->y << endl;
	for (i = path.begin() + 1; i != path.end(); ++i)
	{
		//myfile << "JUMP_TO_AX  " << i->x << "  " << i->y << "  " << i->z << endl;
		myfile << "PTP_AX " << i->x << "  " << i->y << endl;
	}
	myfile << "EndProgramFile" << endl;
	myfile.close();
}

/*
double addNoise(double x, double mean, double stddev)
{
	Random r = new Random();
	double noise = stddev*r.nextGaussian() + mean;
	return x + noise;
}
*/
