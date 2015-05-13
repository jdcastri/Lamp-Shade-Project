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

// split string into vector at every delimiter
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

Mesh mout;

/*
 * Runs octree optimization
 * (x,y,z) : coordinate of bottom left voxel
 * len : number of voxels in voxel cube
 * @return : boolean if entire voxel cube in part of lampshade
 */
bool octTree(int len, int x, int y, int z) {
    Mesh box;
    double spacing = g_voxelGrid->m_spacing;
    CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);

    // when we are considering a voxel cube of length 1, we are only considering one voxel
    if (len == 1) {
        // uncomment bottom line if displaying floor
        return g_voxelGrid->isInside(x,y,z);
        // return g_voxelGrid->isInside(x + offsetX,y + offsetY,z + offsetZ);
    } 

    int hlen = len/2;    

    // get true/false in/out value of all 8 sub-voxels in current voxel cube
    array<bool, 8> children {
        octTree(hlen, x, y, z),
        octTree(hlen, x, y, z+hlen),
        octTree(hlen, x, y+hlen, z),
        octTree(hlen, x, y+hlen, z+hlen),
        octTree(hlen, x+hlen, y, z),
        octTree(hlen, x+hlen, y, z+hlen),
        octTree(hlen, x+hlen, y+hlen, z),
        octTree(hlen, x+hlen, y+hlen, z+hlen)
    };

    // if all children make up lampshade
    array<bool, 8> allTrue {true,true,true,true,true,true,true,true};
    if (children == allTrue) {
        return true;
    }

    // if at least one child is not in lampshade, make all the ones that are part of the voxelization
    for (int i=0; i<children.size(); i++) {
        if (children[i] == true) {
            int child_x = x + hlen*(i/4);
            int child_y = y + hlen*((i/2)%2);
            int child_z = z + hlen*(i%2);
            
            int numHalfSpacings = ((hlen*2)-1);
            CompFab::Vec3 coord(double(child_x)*spacing, double(child_y)*spacing, 
                    double(child_z)*spacing);
            CompFab::Vec3 box0 = coord - hspacing;
            CompFab::Vec3 box1 = coord + CompFab::Vec3(numHalfSpacings*hspacing.m_x, numHalfSpacings*hspacing.m_y, numHalfSpacings*hspacing.m_z);
            makeCube(box, box0, box1);
            mout.append(box);
        }
    }

    return false;
}

// Saves voxel data to OBJ
void saveVoxelsToObj(const char * outfile)
{    
    // run Octree optimization
    octTree(dimMesh,0,0,0);
    mout.save_obj(outfile);
}

//uncomment this version of saveVoxelsToObj if you want to display floor/walls
// Saves voxel data to OBJ
// void saveVoxelsToObj(const char * outfile)
// {
 
//     Mesh box;
//     Mesh mout;
//     int nx = g_voxelGrid->m_dimX;
//     int ny = g_voxelGrid->m_dimY;
//     int nz = g_voxelGrid->m_dimZ;
//     double spacing = g_voxelGrid->m_spacing;
    
//     CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);
    
//     for (int ii = 0; ii < nx; ii++) {
//         for (int jj = 0; jj < ny; jj++) {
//             for (int kk = 0; kk < nz; kk++) {
//                 if(!g_voxelGrid->isInside(ii,jj,kk)){
//                     continue;
//                 }
//                 CompFab::Vec3 coord(((double)ii)*spacing, ((double)jj)*spacing, ((double)kk)*spacing);
//                 CompFab::Vec3 box0 = coord - hspacing;
//                 CompFab::Vec3 box1 = coord + hspacing;
//                 makeCube(box, box0, box1);
//                 mout.append(box);
//             }
//         }
//     }

//     mout.save_obj(outfile);
// }

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
        cout << "Reading input file" << endl;
		getline(myfile, line);
		vector<string> bbMin_params = split(line, ',');
		CompFab::Vec3 bbMin (stoi(bbMin_params[0]), stoi(bbMin_params[1]), stoi(bbMin_params[2]));

		getline(myfile, line);
		double spacing = stod(line);

		getline(myfile,line);
		dimMesh = stoi(line);
        dimRoomX = 256;
        dimRoomY = 256;
		dimRoomZ = 256; // height of room

		// Load shadow image
	    std::string floor_imagePath = "/home/jdcastri/Spring2015/6.S079/project/shadowimages/star.png";
        std::string wall_imagePath = "/home/jdcastri/Spring2015/6.S079/project/shadowimages/squares.png";
        std::string wall_imagePath_2 = "/home/jdcastri/Spring2015/6.S079/project/shadowimages/smiley.png";

        vector<Wall*> walls;
        // add Walls here...
        walls.push_back(new Wall(floor_imagePath, CompFab::Vec3(0,0,1), dimRoomX, dimRoomY, dimRoomZ));
        // walls.push_back(new Wall(wall_imagePath, CompFab::Vec3(0,1,0), dimRoomX, dimRoomY, dimRoomZ));
        // walls.push_back(new Wall(wall_imagePath_2, CompFab::Vec3(1,0,0), dimRoomX, dimRoomY, dimRoomZ));

	    offsetX = dimRoomX/2-dimMesh/2; 
		offsetY = dimRoomY/2-dimMesh/2; 
		offsetZ = dimRoomZ-dimMesh;
 
        if (offsetX < 0 || offsetY < 0 || offsetZ < 0) {
            cout << "ERROR: An offset is negative." << endl;
            return 0;
        }

        // uncomment line below if you want to display floor
		// makeVoxelGrid(bbMin, dimRoomX, dimRoomY, dimRoomZ, spacing);
        makeVoxelGrid(bbMin, dimMesh, dimMesh, dimMesh, spacing);

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
                // uncomment bottom line if displaying floor
				// g_voxelGrid->isInside(ii+offsetX, jj+offsetY, kk+offsetZ) = true;
                g_voxelGrid->isInside(ii, jj, kk) = true;
            }
		}

		//Write out voxel data as OBJ
		std::cout << "Saving Voxels to OBJ...\n";
		saveVoxelsToObj(argv[2]);

		delete g_voxelGrid;	
		myfile.close();
	} else {
		cout << "Unable to open file\n";
	}
}