/******************************************************************************
    file:       main.cpp
    created:
******************************************************************************/
#include <iostream>
#include <fstream>
#include <windows.h>
#include "Box.h"
#include "VisibilityGraph.h"


using namespace std;

void write_program_file(vector<Point> path, string filename);

/*****************************************************************************
 *  main
 */
int main(void)
{

    const int nHind = 3;    // Anzahl der Hindernisse
    Box aHindernis[nHind];  // Unsere Hindernisse

    // Hindernisse initialisieren
    // Hierbei wird für jedes Hinderniss die Größe ( Skalierung ) und
    // die Position im Raum gesetzt
    aHindernis[0].Scale(0.2f, 0.2f, 0.1f);  // QUADER1   0.20000    0.2000    0.10000
    aHindernis[0].Set(0.1f, 0.1f, 0.0f);    // REFPOS   0.200000   0.2000000    0.0000000    0.0000000    0.0000000    0.0000000

    aHindernis[1].Scale(0.2f, 0.2f, 0.1f);  // QUADER2   0.2000    0.20000    0.10000
    aHindernis[1].Set(0.2f, 0.4f, 0.0f);    // REFPOS   0.200000   0.4000000    0.0000000    0.0000000    0.0000000    0.0000000

    aHindernis[2].Scale(0.2f, 0.2f, 0.1f);  // QUADER3   0.2000    0.20000    0.10000
    aHindernis[2].Set(0.3f, 0.7f, 0.0f);    // REFPOS   0.200000   0.4000000    0.0000000    0.0000000    0.0000000    0.0000000

    // Roboter initialisieren
    Point startpoint(0.2, 0., 0.);
   Point endpoint(0.3, 0.7, 0.); // End point for the first two obstacles
    //Point endpoint(0.4, 1.0, 0.);   // End point for the three obstacles

	//Point endpoint(0.2, 0.2,0); //unreachable goal
	//Point endpoint(0.2, 0., 0.); // start and goal are equal

	//Point endpoint(1, 0, 0); // no obstical

	//Point endpoint(0.1, 0.2, 0);


    vector<Point> path;             // create a point vector for storing the path
    Graph g(nHind * 4 + 2);         // create a graph with appropriate number of vertices

    // store all rectangle points in g[2].pt onwards, for each rectangle 4 vertices
    for (int i = 0; i < nHind; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            g[i * 4 + j].pt = aHindernis[i].GetVertex(j);
        }
    }	

    g[nHind * 4].pt = startpoint;   // store startpoint in graph g "behind" the rectangle vertices
    g[nHind * 4 + 1].pt = endpoint; // store endpoint in graph g "behind" the rectangle vertices

    // Startzeit
    DWORD dwStart = GetTickCount();

    path = VisibilityGraph(g, nHind);

    // Zeit für das Aufstellen des Konfigurationsraumes ausgeben ( in ms )
    DWORD dwElapsed = GetTickCount() - dwStart;
    cout << "\nBerechnung dauerte " << dwElapsed << " ms\n";

    write_program_file(path, "VisibilityGraph.prg");

    return 0;
}

/**************************************************************************/
// Ausgabe einer EASYROB *.prg Datei zur Simulation der Ergebnisse und Verifikation
void write_program_file(vector<Point> path, string filename)
{
    ofstream myfile;
    myfile.open(filename);
    myfile << "ProgramFile" << endl;
    //<< "EndInit" << endl << "SPEED_PTP_OV   80.0000" << endl << "ACCEL_PTP_OV   100.0000" << endl
    //<< "OV_PRO         100.0000" << endl;
    
	for(int i=0;i<path.size();i++)
	//why is it reading the path in reverse order ?????????????????????????????????
	//for (int i = (int)path.size()-1; i >= 0; --i)
    {
        myfile << "PTP_AX " << path.at(i).x << "  " << path.at(i).y << endl;
        //myfile << "JUMP_TO_AX " << path.at(i).x << "  " << path.at(i).y << endl;
    }
    myfile << "EndProgramFile" << endl;
    myfile.close();
}


