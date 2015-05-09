#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "../include/CompFab.h"
#include "../include/Mesh.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

CompFab::VoxelGrid *g_voxelGrid; // Voxel grid of universe
unsigned int dimMesh, dimRoomX, dimRoomY, dimRoomZ; // Dimensions of mesh and room
int* imageArray; // Array of pixels of desired projection image
CompFab::Vec3 lightSource; // Location of light source from bottom left corner of room
int offsetX, offsetY, offsetZ; // Location of lamp from bottom left corner of room

using namespace std;

int main(int argc, char **argv) {
	string line;
 	ifstream myfile (argv[1]);
	if (myfile.is_open())
	{
		getline(myfile,line);
		dimMesh = line;
		getline(myfile,line);
		dimRoomX = line;
		getline(myfile,line);
		dimRoomY = line;
		getline(myfile,line);
		dimRoomZ = line;
		cout << dimMesh << endl;
		// cout << dimRoomX << endl;
		// cout << dimRoomY << endl;
		// cout << dimRoomZ << endl;

		while (getline(myfile, line)) {
			//cout << line << endl;
		}
		
		myfile.close();
	} else 
		cout << "Unable to open file\n";
}