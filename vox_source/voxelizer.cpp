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
#include <fstream>

// Global Variables //
std::vector<CompFab::Triangle> g_triangleList; // Triangle list 
CompFab::VoxelGrid *g_voxelGrid; // Voxel grid of universe
unsigned int dimMesh; // Dimensions of mesh and room

// write to file: bbmin, spacing, dimMesh, all voxels in lampshade mesh as (ii,jj,kk) coordinates
std::ofstream file; 

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

// Loads input mesh and initializes voxel grid of room
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

    g_voxelGrid = new CompFab::VoxelGrid(bbMin-hspacing, dimMesh, dimMesh, dimMesh, spacing);
    file << bbMin.m_x << ',' << bbMin.m_y << ',' << bbMin.m_z << '\n';
    file << spacing << '\n';

    delete tempMesh;
    
    return true;
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

                    file << ii << "," << jj << "," << kk << "\n";


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
    if(argc < 2){
        std::cout<<"Usage: Voxelizer InputMeshFilename MeshDimension \n";
        return 0;
    }

    file.open("voxelized16.txt");

    // Record start time
    time_t start = time(0);

    // Parameters //
    dimMesh = std::stoi(argv[2]); // dimensions of mesh (affects runtime)

    // Load OBJ file of lampshade
    std::cout << "Load Mesh: " << argv[1] << "\n";
    loadMesh(argv[1]);

    file << dimMesh << "\n";   

    // Ray casting for Voxelization
    std::cout << "Voxelizing...\n";
    int numThreads = 4;
    parallelVoxelization(numThreads);
    
    delete g_voxelGrid;
    file.close();

    // Record time and calculate runtime
    time_t end = time(0);
    double runtime = difftime(end, start);

    std::cout << "Total Runtime: " << runtime << " seconds \n";
}
