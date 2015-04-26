//Computational Fabrication Assignment #1
// By David Levin 2014
#include <iostream>
#include <vector>
#include <cmath>
#include "../include/CompFab.h"
#include "../include/Mesh.h"

// opencv dependencies
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//Ray-Triangle Intersection
//Returns 1 if triangle and ray intersect, 0 otherwise
int rayTriangleIntersection(CompFab::Ray &ray, CompFab::Triangle &triangle)
{
    
    CompFab::Vec3 v1 = triangle.m_v1;
    CompFab::Vec3 v2 = triangle.m_v2;
    CompFab::Vec3 v3 = triangle.m_v3;
    CompFab::Vec3 orig = ray.m_origin;
    CompFab::Vec3 dir = ray.m_direction;

    CompFab::Vec3 edge1 = v2 - v1;
    CompFab::Vec3 edge2 = v3 - v1;
    CompFab::Vec3 P = dir % edge2;
    double determinant = edge1 * P;

    if (std::abs(determinant) < .000001 ) {
        return 0;
    }

    // calculate u. Assure u is between (0,1)

    double inv_det = 1.0f / determinant;
    CompFab::Vec3 T = orig - v1;

    float u = (T * P) * inv_det;

    if ( u < 0 || u > 1) {
        return 0;
    }

    // calculate v. Assure v is positive and u+v<1
    CompFab::Vec3 Q = T % edge1;
    double v = (dir * Q) * inv_det;

    if ( v < 0 || u + v > 1 ) {
        return 0;
    }

    // calculate t
    double t = (edge2 * Q) * inv_det;

    if ( t > .000001) {
        return 1;
    }

    return 0;
}

//Triangle list (global)
typedef std::vector<CompFab::Triangle> TriangleList;

TriangleList g_triangleList;
CompFab::VoxelGrid *g_voxelGrid;

// Number of intersections with surface made by a ray originating at voxel and cast in direction.
int numSurfaceIntersections(CompFab::Vec3 &voxelPos, CompFab::Vec3 &dir)
{
    unsigned int numHits = 0;

    CompFab::Ray ray (voxelPos, dir);
    
    for (int i=0; i<g_triangleList.size(); i++) {
        numHits += rayTriangleIntersection(ray, g_triangleList[i]);
    }
    
    return numHits;
}

bool loadMesh(char *filename, unsigned int dimMesh, unsigned int dimRoom)
{
    g_triangleList.clear();
    
    Mesh *tempMesh = new Mesh(filename, true);
    
    CompFab::Vec3 v1, v2, v3;

    //copy triangles to global list
    for(unsigned int tri =0; tri<tempMesh->t.size(); ++tri)
    {
        v1 = tempMesh->v[tempMesh->t[tri][0]];
        v2 = tempMesh->v[tempMesh->t[tri][1]];
        v3 = tempMesh->v[tempMesh->t[tri][2]];
        g_triangleList.push_back(CompFab::Triangle(v1,v2,v3));
    }

    //Create Voxel Grid
    CompFab::Vec3 bbMax, bbMin;
    BBox(*tempMesh, bbMin, bbMax);
    
    //Build Voxel Grid
    double bbX = bbMax[0] - bbMin[0];
    double bbY = bbMax[1] - bbMin[1];
    double bbZ = bbMax[2] - bbMin[2];
    double spacing;
    
    if(bbX > bbY && bbX > bbZ)
    {
        spacing = bbX/(double)(dimMesh);
    } else if(bbY > bbX && bbY > bbZ) {
        spacing = bbY/(double)(dimMesh);
    } else {
        spacing = bbZ/(double)(dimMesh);
    }
    
    CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);
    
    g_voxelGrid = new CompFab::VoxelGrid(bbMin-hspacing, dimRoom, dimRoom, dimRoom, spacing);

    delete tempMesh;
    
    return true;
   
}

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


int main(int argc, char **argv)
{

    unsigned int dimMesh = 32; // dimensions of mesh (e.g. 32x32x32)
    unsigned int dimRoom = 128; // dimensions of room 

    // offsets of lamp from bottom left corner of room
    int offsetX = dimRoom/2-dimMesh/2; 
    int offsetY = dimRoom/2-dimMesh/2;
    int offsetZ = dimRoom-dimMesh;

    // load OBJ
    if(argc < 3)
    {
        std::cout<<"Usage: Voxelizer InputMeshFilename OutputMeshFilename \n";
        return 0;
    }
    
    std::cout<<"Load Mesh : "<<argv[1]<<"\n";
    loadMesh(argv[1], dimMesh, dimRoom);


    //Cast ray, check if voxel is inside or outside
    //even number of surface intersections = outside (OUT then IN then OUT)
    // odd number = inside (IN then OUT)
    CompFab::Vec3 voxelPos;
    CompFab::Vec3 direction(1.0,0.0,0.0);

    double spacing = g_voxelGrid->m_spacing;
    
    CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);
    
    for (int ii = 0; ii < dimMesh; ii++) {
        for (int jj = 0; jj < dimMesh; jj++) {
            for (int kk = 0; kk < dimMesh; kk++) {
                CompFab::Vec3 coord(((double)ii)*spacing, ((double)jj)*spacing, ((double)kk)*spacing);
                voxelPos = coord + hspacing;
                int hits = numSurfaceIntersections(voxelPos, direction);

                if (hits % 2 != 0) {
                    g_voxelGrid->isInside(ii+offsetX,jj+offsetY,kk+offsetZ) = true;
                }
            }
        }
    }

    using namespace cv;
    using namespace std;

    // Reads image at path.
    Mat img = imread("/home/jdcastri/Dropbox (MIT)/Spring 2015/6.S079/project/input/images/default_square.png",CV_LOAD_IMAGE_GRAYSCALE);
    // intensity is value at (y,x) coordinate of image
    Scalar intensity = img.at<uchar>(220, 140);
    // intensity[0] is value between (0,255) for pixel
    // 0 is black. 255 is white. 
    cout << intensity[0] << endl;

    // show a window with the image. For testing purposes.
    imshow("hello", img);
    // halts code
    waitKey(0);

    // Show floor
    for (int ii = 0; ii < dimRoom; ii++) {
        for (int jj = 0; jj < dimRoom; jj++) {
            g_voxelGrid->isInside(ii,jj,0) = true;
        }
    }



    //Write out voxel data as obj
    saveVoxelsToObj(argv[2]);
    
    delete g_voxelGrid;
}
