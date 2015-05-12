#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "../include/CompFab.h"
#include "../include/Mesh.h"
#include "../include/Wall.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

CompFab::VoxelGrid *g_voxelGrid; // Voxel grid of universe
unsigned int dimMesh, dimRoomX, dimRoomY, dimRoomZ; // Dimensions of mesh and room
int* imageArray; // Array of pixels of desired projection image
CompFab::Vec3 lightSource; // Location of light source from bottom left corner of room
int offsetX, offsetY, offsetZ; // Location of lamp from bottom left corner of room

using namespace std;

vector<string> split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str); // Turn the string into a stream.
	string tok;

	while(getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}

	return internal;
}

void makeVoxelGrid(CompFab::Vec3 &bbMin, int dimRoomX, int dimRoomY, int dimRoomZ, double spacing) {
	CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);
	g_voxelGrid = new CompFab::VoxelGrid(bbMin-hspacing, dimRoomX, dimRoomY, dimRoomZ, spacing);
}

// Saves voxel data to OBJ
void saveVoxelsToObj(const char * outfile)
{
 
    Mesh box;
    Mesh mout;
    int nx = g_voxelGrid->m_dimX;
    int ny = g_voxelGrid->m_dimY;
    int nz = g_voxelGrid->m_dimZ;
    double spacing = g_voxelGrid->m_spacing;
    
    CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);
    
    for (int ii = 0; ii < nx; ii++) {
        for (int jj = 0; jj < ny; jj++) {
            for (int kk = 0; kk < nz; kk++) {
                if(!g_voxelGrid->isInside(ii,jj,kk)){
                    continue;
                }
                CompFab::Vec3 coord(((double)ii)*spacing, ((double)jj)*spacing, ((double)kk)*spacing);
                CompFab::Vec3 box0 = coord - hspacing;
                CompFab::Vec3 box1 = coord + hspacing;
                makeCube(box, box0, box1);
                mout.append(box);
            }
        }
    }

    mout.save_obj(outfile);
}

int main(int argc, char **argv) {

	// Validate arguments
    if(argc < 2){
        std::cout<<"Usage: ./lampshade parameters.txt OutputMeshFilename \n";
        return 0;
    }

	string line;
 	ifstream myfile (argv[1]);
	if (myfile.is_open())
	{
		getline(myfile, line);
		vector<string> bbMin_params = split(line, ',');
		CompFab::Vec3 bbMin (stoi(bbMin_params[0]), stoi(bbMin_params[1]), stoi(bbMin_params[2]));

		getline(myfile, line);
		double spacing = stod(line);

		getline(myfile,line);
		dimMesh = stoi(line);
        dimRoomX = 300;
        dimRoomY = 300;
		dimRoomZ = 300; // height of room

		// Load shadow image
	    std::string floor_imagePath = "/home/jdcastri/Spring2015/6.S079/project/shadowimages/pattern1.png";
        std::string wall_imagePath = "/home/jdcastri/Spring2015/6.S079/project/shadowimages/star.png";
	    // std::string imagePath = "../shadowimages/1_notsq.png";

        vector<Wall*> walls;
        // add Walls here...
        walls.push_back(new Wall(floor_imagePath, CompFab::Vec3(0,0,1), dimRoomX, dimRoomY, dimRoomZ));
        walls.push_back(new Wall(wall_imagePath, CompFab::Vec3(0,-1,0), dimRoomX, dimRoomY, dimRoomZ));

	    int offsetX = dimRoomX/2-dimMesh/2; 
		int offsetY = dimRoomY/2-dimMesh/2; 
		int offsetZ = dimRoomZ-dimMesh;
 
        if (offsetX < 0 || offsetY < 0 || offsetZ < 0) {
            cout << "ERROR: An offset is negative." << endl;
            return 0;
        }

		makeVoxelGrid(bbMin, dimRoomX, dimRoomY, dimRoomZ, spacing);

		cout << "Removing voxels" << endl;
		while (getline(myfile, line)) {
			vector<string> vox_params = split(line, ',');
			int ii = stoi(vox_params[0]);
			int jj = stoi(vox_params[1]);
			int kk = stoi(vox_params[2]);

			// Light source location in room frame
    		lightSource = CompFab::Vec3(dimRoomX/2, dimRoomY/2, dimRoomZ - dimMesh/3);

            CompFab::Vec3 roomVoxelPos(((double)(ii+offsetX)), 
                    ((double)(jj+offsetY)), 
                    ((double)(kk+offsetZ)));


            bool blockVox = true;
            for (int i=0; i<walls.size(); i++) {
                blockVox = blockVox && walls.at(i)->shouldBlock(roomVoxelPos, lightSource);
                // if a wall doesn't want to block a voxel, the output of the other walls are irrelevant
                if (!blockVox) { break; }
            }

			if(blockVox){
				g_voxelGrid->isInside(ii+offsetX, jj+offsetY, kk+offsetZ) = true;
            }
		}

        // // Show floor with image
		// for (int ii = 0; ii < dimRoomX; ii++) {
		//     for (int jj = 0; jj < dimRoomY; jj++) {
		//         g_voxelGrid->isInside(ii,jj,0) = 1-floor.imageArray[jj*dimRoomX + ii];
		//     }
		// }

		//Write out voxel data as OBJ
		std::cout << "Saving Voxels to OBJ...\n";
		saveVoxelsToObj(argv[2]);

		delete g_voxelGrid;
		// delete[] imageArray;		
		myfile.close();
	} else {
		cout << "Unable to open file\n";
	}
}