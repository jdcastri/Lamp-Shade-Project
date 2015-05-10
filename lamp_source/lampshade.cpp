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

vector<string> split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str); // Turn the string into a stream.
	string tok;

	while(getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}

	return internal;
}

// Ray-Floor Intersection Point
// Returns the point where the given ray intersects with the floor PLANE (even if out of bounds).
CompFab::Vec3 rayFloorIntersection(CompFab::Ray &ray)
{
    CompFab::Vec3 orig, dir, floorNorm, intersection;

    orig = ray.m_origin;
    dir = ray.m_direction;

    // Normal to floor is (0,0,1)
    floorNorm = CompFab::Vec3(0.0,0.0,1.0);

    // Time of intersection
    double t = -1 * (orig * floorNorm) / (dir*floorNorm);

    return CompFab::Vec3(orig[0]+t*dir[0], orig[1]+t*dir[1], orig[2]+t*dir[2]);
}

// Returns whether or not the voxel should still be present to block light to floor.
int shouldBlock(CompFab::Vec3 &meshVoxelPos, CompFab::Vec3 &lightSourcePos)
{
    CompFab::Vec3 intersection, dir;

    dir = meshVoxelPos - lightSourcePos;
    CompFab::Ray ray(lightSourcePos,dir); // ray from light source pointing toward voxel

    intersection = rayFloorIntersection(ray);

    if(intersection[0] < 0 || intersection[1] < 0 || 
        intersection[0] > dimRoomX - 1 || intersection[1] > dimRoomY - 1 ){
        return 1; // If intersection is outside room dimensions, voxel should stay.
    }

    // imageArray tells us whether or not that pixel on the ground is black or white.
    // We treat 0 as shadow and 1 as light
    // If pixel is 0, we want a shadow so we should block (return 1), and vice versa.
    return 1-imageArray[int(intersection[1])*dimRoomX + int(intersection[0])];
}

void makeVoxelGrid(CompFab::Vec3 &bbMin, int dimRoomX, int dimRoomY, int dimRoomZ, double spacing) {
	CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);
	g_voxelGrid = new CompFab::VoxelGrid(bbMin-hspacing, dimRoomX, dimRoomY, dimRoomZ, spacing);
}

// Inserts 0's and 1's into imageArray based on loaded image from imagePath.
void loadImage(std::string imagePath){
    cv::Mat img = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE); // Reads image at path.
    if (! img.data) {
        std::cout << "***********Could not open or find image**********\n" << std::endl;
        throw 20; 
    }

    dimRoomX = int(img.cols);
    dimRoomY = int(img.rows);

    cv::Scalar intensity;

    // Image pixels
    imageArray = new int[dimRoomX*dimRoomY]; // to access (x,y): imageArray[y*dimRoomX + x]
    
    for(int x = 0; x < dimRoomX; x++){
        for(int y = 0; y < dimRoomY; y++){
            intensity = img.at<uchar>(y, x);
            // intensity[0] is value between (0,255) for pixel
            // 0 is black. 255 is white.
            if(intensity[0] < 128){
                imageArray[y*dimRoomX+x] = 0; 
            }
            else{
                imageArray[y*dimRoomX+x] = 1;
            }
        }
    }
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
		// set first six lines
		getline(myfile, line);
		vector<string> bbMin_params = split(line, ',');
		CompFab::Vec3 bbMin (stoi(bbMin_params[0]), stoi(bbMin_params[1]), stoi(bbMin_params[2]));

		getline(myfile, line);
		double spacing = stod(line);

		getline(myfile,line);
		dimMesh = stoi(line);
		dimRoomZ = 100; // height of room

		// Load shadow image
	    std::string imagePath = "/home/jdcastri/Spring2015/6.S079/project/shadowimages/star.png";
	    // std::string imagePath = "../shadowimages/1_notsq.png";
	    std::cout << "Load Image: " << imagePath << "\n";
	    loadImage(imagePath);

	    int offsetX = dimRoomX/2-dimMesh/2; 
		int offsetY = dimRoomY/2-dimMesh/2; 
		int offsetZ = dimRoomZ-dimMesh;

		makeVoxelGrid(bbMin, dimRoomX, dimRoomY, dimRoomZ, spacing);

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

			if(shouldBlock(roomVoxelPos, lightSource)){
				g_voxelGrid->isInside(ii+offsetX,jj+offsetY,kk+offsetZ) = true;
            }
		}

	    // Show floor with image
		// Delete later
		for (int ii = 0; ii < dimRoomX; ii++) {
		    for (int jj = 0; jj < dimRoomY; jj++) {
		        g_voxelGrid->isInside(ii,jj,0) = 1-imageArray[jj*dimRoomX + ii];
		    }
		}

		//Write out voxel data as OBJ
		std::cout << "Saving Voxels to OBJ...\n";
		saveVoxelsToObj(argv[2]);

		delete g_voxelGrid;
		delete[] imageArray;		
		myfile.close();
	} else {
		cout << "Unable to open file\n";
	}
}