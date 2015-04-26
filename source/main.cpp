/*
** Authors: Sami Alsheikh and Juan Castrillon
** April 2015
** Based on work by David Levin 2014 for 6.S079 Assignment 1.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include "../include/CompFab.h"
#include "../include/Mesh.h"

// OpenCV dependencies
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// Global Variables //
std::vector<CompFab::Triangle> g_triangleList; // Triangle list 
CompFab::VoxelGrid *g_voxelGrid; // Voxel grid of universe
unsigned int dimMesh = 32; // dimensions of mesh (affects runtime)
unsigned int dimRoom = 256; // dimensions of room (does not affect runtime)
// NOTE: As code is written now, dimRoom must equal dimension of input image.
// TODO Make scaling/room and mesh dimensions not scale in relation to each other.
int* imageArray; // Array of pixels of desired projection image


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
        intersection[0] > dimRoom - 1 || intersection[1] > dimRoom - 1 ){
        return 1; // If intersection is outside room dimensions, voxel should stay.
    }

    // imageArray tells us whether or not that pixel on the ground is black or white.
    // We treat 0 as shadow and 1 as light
    // If pixel is 0, we want a shadow so we should block (return 1), and vice versa.
    return 1-imageArray[int(intersection[0])*dimRoom + int(intersection[1])];
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


// Loads floor voxels and imageArray with test smiley face
// Used for testing before we could load images.
void loadTestSmileyOnGround()
{

    // Include holes in floor to resemble picture
    int x,y;
    // EYE //
    for(int dx = 0; dx < 0.125*dimRoom; dx++){
        for(int dy = 0; dy < 0.125*dimRoom; dy++){
            x = dimRoom/3.0+dx;
            y = dimRoom/3.0+dy;
            g_voxelGrid->isInside(x,y,0) = false;
            imageArray[x*dimRoom+y] = 0;
        }
    }

    // EYE // 
    for(int dx = 0; dx < 0.125*dimRoom; dx++){
        for(int dy = 0; dy < 0.125*dimRoom; dy++){
            x = 3*dimRoom/4.0+dx;
            y = dimRoom/3.0+dy;
            g_voxelGrid->isInside(x,y,0) = false;
            imageArray[x*dimRoom+y] = 0;
        }
    }

    // MOUTH //
    for(int dy = 2*dimRoom/3; dy < 2*dimRoom/3+dimRoom/10; dy++ ){
        for(float t = 0; t < 3.14; t=t+6.0/dimRoom){
            x = dimRoom/2.75+t*dimRoom/6;
            y = dy+20*sin(t);
            g_voxelGrid->isInside(x,y,0) = false;
            imageArray[x*dimRoom+y] = 0;
        }
    }

}

// Inserts 0's and 1's into imageArray based on loaded image from imagePath.
void loadImage(std::string imagePath){
    cv::Mat img = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE); // Reads image at path.
    cv::Scalar intensity;

    // Image pixels
    imageArray = new int[dimRoom*dimRoom]; // to access (x,y): imageArray[x*dimRoom + y]
    
    for(int x = 0; x < dimRoom; x++){
        for(int y = 0; y < dimRoom; y++){
            intensity = img.at<uchar>(y, x);
            // intensity[0] is value between (0,255) for pixel
            // 0 is black. 255 is white.
            if(intensity[0] < 128){
                imageArray[x*dimRoom+y] = 0; 
            }
            else{
                imageArray[x*dimRoom+y] = 1;
            }
        }
    }
}

int main(int argc, char **argv)
{

    // Load OBJ file of lampshade
    if(argc < 3)
    {
        std::cout<<"Usage: Voxelizer InputMeshFilename OutputMeshFilename \n";
        return 0;
    }

    std::cout << "Load Mesh: " << argv[1] << "\n";
    loadMesh(argv[1], dimMesh, dimRoom);

    // Load image
    std::string imagePath = "../shadowimages/star.png";
    loadImage(imagePath);

    std::cout << "Image from " << imagePath << " loaded" << "\n";

    // Show floor with image
    // Delete later
    for (int ii = 0; ii < dimRoom; ii++) {
        for (int jj = 0; jj < dimRoom; jj++) {
            g_voxelGrid->isInside(ii,jj,0) = 1-imageArray[ii*dimRoom + jj];
        }
    }

    // Delete later
    // loadTestSmileyOnGround();

    /// Configure light source and lampshade ///
    // Offsets of lamp from bottom left corner of room
    int offsetX = dimRoom/2-dimMesh/2; 
    int offsetY = dimRoom/2-dimMesh/2; 
    int offsetZ = dimRoom-dimMesh;
    // Light source location in room frame
    CompFab::Vec3 lightSource(dimRoom/2, dimRoom/2, dimRoom - dimMesh/3);


    /// Ray casting for voxelization ///
    CompFab::Vec3 voxelPos, roomVoxelPos;
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

    // Show light source location with single voxel
    // Delete later
    // g_voxelGrid->isInside(lightSource[0],lightSource[1],lightSource[2]) = true;

    //Write out voxel data as obj
    saveVoxelsToObj(argv[2]);
    
    delete g_voxelGrid;
    delete[] imageArray;
}
