// TXTY_2ObstaclesPotential.cpp : Defines the entry point for the console application.
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
    const int nHind = 2;    // Anzahl der Hindernisse
    const int nRob = 1;     // Anzahl der Roboterglieder
    vector<Point> path;     // Create a vector containing integers
    Box aHindernis[nHind];  // Unsere Hindernisse
    Box Roboter[nRob];      // Roboter

    Potential pot1("Potential1");

    // Hindernisse initialisieren
    // Hierbei wird für jedes Hinderniss die Größe ( Skalierung ) und die Position im Raum gesetzt
    aHindernis[0].Scale(0.20f, 0.20f, 0.10f);   // QUADER1   0.20000    0.2000    0.10000
    aHindernis[0].Set(0.1f, 0.1f, 0.0f);        // REFPOS   0.200000   0.2000000    0.0000000    0.0000000    0.0000000    0.0000000

    aHindernis[1].Scale(0.20f, 0.20f, 0.10f);   // QUADER2   0.2000    0.20000    0.10000
    aHindernis[1].Set(0.3f, 0.3f, 0.0f);        // REFPOS   0.200000   0.4000000    0.0000000    0.0000000    0.0000000    0.0000000

    // Roboter initialisieren
    Roboter[0].Scale(0.05f, 0.05f, 0.20f);      // QUADER    0.05000    0.05000    0.20000
    //Roboter[0].Translate( -0.025f, -0.025f, 0.0f ); // REFPOS   -0.02500000   -0.02500000    0.0000000    0.0000000    0.0000000    0.0000000

    // Startzeit
    DWORD dwStart = GetTickCount();

    // Initialize start, goal, actPoint and heading
    //pot1.setStartPosition(0.6, 0.0); // local minimum
    //pot1.setStartPosition(0.2, -0.2); // local minimum
    pot1.setStartPosition(-0.1, -0.2); // no local minimum

    pot1.setGoalPosition(0.1, 0.4); // EASYROB

    Roboter[0].Set(pot1.getStartPosition());

    pot1.setActPoint(pot1.getStartPosition());

    path.push_back(pot1.getStartPosition());

    Point robPos;
    bool goal_reached = false;
    bool local_minimum_reached = false;

    while (!goal_reached && !local_minimum_reached)
    {
        goal_reached = pot1.update_box(aHindernis, Roboter, nHind);
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

	while (true) {

	}

    return 0;
}

/****************************************************************************************/
// Chekcs, if local minimum is reached
bool check_local_minimum(vector<Point> path, Point act)
{
    //simple check if the current position was reached bevore

	double errorMargin = 0.00005;

	for (int i = 0; i < path.size()-1; i++)
	{

		double xDiff = abs(act.x - path[i].x);
		double yDiff = abs(act.y - path[i].y);

		if (xDiff <= errorMargin && yDiff<= errorMargin) {
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
    myfile.open("Potential.prg");
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
