/* Authors: Sami Alsheikh and Juan Castrillon
 * April 2015
 * Based on work by David Levin 2014 for 6.S079 Assignment 1. 
 */
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include "../include/CompFab.h"
#include "../include/Mesh.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"



// Global Variables //
std::vector<CompFab::Triangle> g_triangleList; // Triangle list 
CompFab::VoxelGrid *g_voxelGrid; // Voxel grid of universe
unsigned int dimMesh, dimRoomX, dimRoomY, dimRoomZ; // Dimensions of mesh and room
int* imageArray; // Array of pixels of desired projection image
CompFab::Vec3 lightSource; // Location of light source from bottom left corner of room
int offsetX, offsetY, offsetZ; // Location of lamp from bottom left corner of room



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

// Ray-Triangle Intersection
// Returns 1 if triangle and ray intersect, 0 otherwise
int rayTriangleIntersection(CompFab::Ray &ray, CompFab::Triangle &triangle)
{
    CompFab::Vec3 v1, v2, v3, orig, dir, edge1, edge2, P;
    float determinant, inv_det, u, v, t;

    v1 = triangle.m_v1;
    v2 = triangle.m_v2;
    v3 = triangle.m_v3;
    orig = ray.m_origin;
    dir = ray.m_direction;

    edge1 = v2 - v1;
    edge2 = v3 - v1;
    P = dir % edge2;
    determinant = edge1 * P;

    if (std::abs(determinant) < .000001 ) {
        return 0;
    }

    // Calculate u. Assert that u is between (0,1)
    inv_det = 1.0f / determinant;
    CompFab::Vec3 T = orig - v1;

    u = (T * P) * inv_det;

    if ( u < 0 || u > 1) {
        return 0;
    }

    // Calculate v. Assure v is positive and u+v<1
    CompFab::Vec3 Q = T % edge1;
    v = (dir * Q) * inv_det;

    if ( v < 0 || u + v > 1 ) {
        return 0;
    }

    // Calculate t
    t = (edge2 * Q) * inv_det;

    if ( t > .000001) {
        return 1;
    }

    return 0;
}

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

// Loads input mesh and initializes voxel grid of room of room
bool loadMesh(char *filename)
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

    g_voxelGrid = new CompFab::VoxelGrid(bbMin-hspacing, dimRoomX, dimRoomY, dimRoomZ, spacing);

    delete tempMesh;
    
    return true;
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

// Ray casting for voxelizations with specified bounds
void voxelization(int starti, int endi, int startj, int endj, int startk, int endk){
    CompFab::Vec3 voxelPos, roomVoxelPos;
    CompFab::Vec3 direction(1.0,0.0,0.0);
 
    double spacing = g_voxelGrid->m_spacing;
    
    CompFab::Vec3 hspacing(0.5*spacing, 0.5*spacing, 0.5*spacing);
    
    for (int ii = starti; ii < endi; ii++) {
        for (int jj = startj; jj < endj; jj++) {
            for (int kk = startk; kk < endk; kk++) {
                CompFab::Vec3 coord(((double)ii)*spacing, ((double)jj)*spacing, ((double)kk)*spacing);
                voxelPos = coord + hspacing;
                int hits = numSurfaceIntersections(voxelPos, direction);

                if (hits % 2 != 0) {
                    // At this point, we would normally set 
                    // voxel to true, but for the desired shadow image,
                    // we check to see if we should leave this
                    // voxel in or out for light to pass through.

                    // Location of voxel in room frame
                    CompFab::Vec3 roomVoxelPos(((double)(ii+offsetX)), 
                                        ((double)(jj+offsetY)), 
                                        ((double)(kk+offsetZ)));

                    if(shouldBlock(roomVoxelPos, lightSource)){
                        g_voxelGrid->isInside(ii+offsetX,jj+offsetY,kk+offsetZ) = true;
                    }
                }
            }
        }
    }
}

// Run voxelization in parallel on numThreads threads.
// Loses benefits if greater than number of cores (e.g. 2,3, or 4).
void parallelVoxelization(unsigned int numThreads){
    std::thread *threads = new std::thread[numThreads];
    int chunkSize = dimMesh / numThreads;
    int l = 0;          // left boundary
    int r = chunkSize;  // right boundary

    for(int i = 0; i < numThreads-1; i++){
        threads[i] = std::thread(voxelization, 0, dimMesh, 0, dimMesh, l, r);
        l = r;
        r += chunkSize;
    }

    r = dimMesh;
    threads[numThreads-1] = std::thread(voxelization, 0, dimMesh, 0, dimMesh, l, r);

    // Wait for each thread to finish
    for(int i = 0; i < numThreads; i++){
        threads[i].join(); 
    }

    delete[] threads;
}

int main(int argc, char **argv)
{
    // Validate arguments
    if(argc < 3){
        std::cout<<"Usage: Voxelizer InputMeshFilename OutputMeshFilename \n";
        return 0;
    }

    // Record start time
    time_t start = time(0);


    // Parameters //
    dimMesh = 32; // dimensions of mesh (affects runtime)
    dimRoomZ = 100; // height of room

    
    // Load shadow image
    //std::string imagePath = "/home/jdcastri/Spring2015/6.S079/project/shadowimages/star.png";
    std::string imagePath = "../shadowimages/1_notsq.png";
    std::cout << "Load Image: " << imagePath << "\n";
    loadImage(imagePath);


    // Load OBJ file of lampshade
    std::cout << "Load Mesh: " << argv[1] << "\n";
    loadMesh(argv[1]);


    // Show floor with image
    // Delete later
    for (int ii = 0; ii < dimRoomX; ii++) {
        for (int jj = 0; jj < dimRoomY; jj++) {
            g_voxelGrid->isInside(ii,jj,0) = 1-imageArray[jj*dimRoomX + ii];
        }
    }

    /// Configure light source and lampshade ///
    // Offsets of lamp from bottom left corner of room
    offsetX = dimRoomX/2-dimMesh/2; 
    offsetY = dimRoomY/2-dimMesh/2; 
    offsetZ = dimRoomZ-dimMesh;
    // Light source location in room frame
    lightSource = CompFab::Vec3(dimRoomX/2, dimRoomY/2, dimRoomZ - dimMesh/3);

    // Ray casting for Voxelization
    std::cout << "Voxelizing...\n";
    parallelVoxelization(4);

    // Show light source location with single voxel
    // Delete later
    // g_voxelGrid->isInside(lightSource[0],lightSource[1],lightSource[2]) = true;

    //Write out voxel data as OBJ
    std::cout << "Saving Voxels to OBJ...\n";
    saveVoxelsToObj(argv[2]);
    
    delete g_voxelGrid;
    delete[] imageArray;

    // Record time and calculate runtime
    time_t end = time(0);
    double runtime = difftime(end, start);

    std::cout << "Total Runtime: " << runtime << " seconds \n";
}
