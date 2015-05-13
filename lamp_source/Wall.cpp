#include "../include/Wall.h"
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
/* 
 * Wall is a plane in the room simulation
 * imagePath : image to be projected on wall
 * norm : normal of wall plane
 * dimRoomX,Y,Z : dimensions of entire room
 */
Wall::Wall(std::string imagePath, CompFab::Vec3 norm, int dimRoomX, int dimRoomY, int dimRoomZ){
    normal = norm;
    setAxes(dimRoomX, dimRoomY, dimRoomZ);
    loadImage(imagePath);
};

// set width, height axes based on wall normal
void Wall::setAxes(int dimRoomX, int dimRoomY, int dimRoomZ) {
	// find point on wall by sum of normal components
	if (CompFab::Vec3(1,1,1) * normal > 0) {
		point = CompFab::Vec3(0,0,0);
	} else {
		point = CompFab::Vec3(dimRoomX, dimRoomY, dimRoomZ);
	}

	// find corresponding wall dimensions
	if (normal.m_z == 1) {
		dimWallWidth = dimRoomY;
		dimWallHeight = dimRoomX;
	} else {
		dimWallHeight = dimRoomZ;
		if (normal.m_x == 1 || normal.m_x == -1) {
			dimWallWidth = dimRoomY;
		} else if (normal.m_y == 1 || normal.m_y == -1) {
			dimWallWidth = dimRoomX;
		} else {
			cout << "ERROR: Enter valid normal!" << endl;
			throw 20;
		}
	}
}

// Inserts 0's and 1's into imageArray based on loaded image from imagePath.
void Wall::loadImage(std::string imagePath){
    using namespace cv;
	std::cout << "Loading Wall: " << imagePath << "\n";
    Mat img = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE); // Reads image at path.
    if (! img.data) {
        std::cout << "***********Could not open or find image**********\n" << std::endl;
        throw 20; 
    }

    // set image dimensions to match room dimensions
    // scale and center image relative to wall
    double scaleFactor;
    int paddingWidth, paddingHeight;
    int dimImgHeight = int(img.cols);
    int dimImgWidth = int(img.rows);

    double roomAspectRatio = double(dimWallHeight) / dimWallWidth;
    double imgAspectRatio = double(dimImgHeight) / dimImgWidth;

    if (roomAspectRatio > imgAspectRatio) {
        // scale by width
        scaleFactor = double(dimWallWidth) / dimImgWidth;
        paddingWidth = 0;
        paddingHeight = (dimWallHeight - (dimImgHeight * scaleFactor)) / 2.;
    } else {
    	// scale by height
        scaleFactor = double(dimWallHeight) / dimImgHeight;
        paddingWidth = (dimWallWidth - (dimImgWidth * scaleFactor)) / 2.;
        paddingHeight = 0;
    }

    Mat imgResized;
    Size size(dimImgWidth * scaleFactor, dimImgHeight * scaleFactor);
    resize(img, imgResized, size);

    cv::Scalar intensity;

    // Set image pixels
    imageArray = new int[dimWallWidth*dimWallHeight]; // to access (x,y): imageArray[y*dimWallWidth + x]
    for(int x = 0; x < dimWallHeight; x++){
        for(int y = 0; y < dimWallWidth; y++){
            if (y < paddingWidth || x < paddingHeight || y > paddingWidth + dimImgWidth * scaleFactor || x > paddingHeight + dimImgHeight * scaleFactor) {
                imageArray[y*dimWallHeight + x] = 0;
            } else {
                intensity = imgResized.at<uchar>(y - paddingWidth, x - paddingHeight);
                // intensity[0] is value between (0,255) for pixel
                // 0 is black. 255 is white.
                if(intensity[0] < 128){
                    imageArray[y*dimWallHeight+x] = 0; 
                }
                else{
                    imageArray[y*dimWallHeight+x] = 1;
                }    
            }
        }
    }
};

CompFab::Vec3 Wall::rayWallIntersection(CompFab::Ray &ray) {
    CompFab::Vec3 ray_orig, ray_dir, floorNorm, intersection;

    ray_orig = ray.m_origin;
    ray_dir = ray.m_direction;

    // Time of intersection
    double t = ((point - ray_orig) * normal) / (ray_dir*normal);

    // negative time indicates no intersection
    if (t<0) {
    	return CompFab::Vec3(-1,-1,-1);
    }

    return CompFab::Vec3(ray_orig[0]+t*ray_dir[0], ray_orig[1]+t*ray_dir[1], ray_orig[2]+t*ray_dir[2]);
}

// Returns whether or not the voxel should still be present to block light to floor.
int Wall::shouldBlock(CompFab::Vec3 &meshVoxelPos, CompFab::Vec3 &lightSourcePos) {
    CompFab::Vec3 intersection, dir;

    dir = meshVoxelPos - lightSourcePos;
    CompFab::Ray ray(lightSourcePos,dir); // ray from light source pointing toward voxel

    intersection = rayWallIntersection(ray);

	// find corresponding wall dimensions
	int width, height;
	if (normal.m_z == 1) {
		width = intersection[1];
		height = intersection[0];
	} else {
		height = intersection[2];
		if (normal.m_x == 1 || normal.m_x == -1) {
			width = intersection[1];
		} else if (normal.m_y == 1 || normal.m_y == -1) {
			width = intersection[0];
		} else {
			cout << "ERROR: Enter valid normal!" << endl;
			throw 20;
		}
	}

    // If intersection is outside wall dimensions, voxel should stay.
    if(width < 0 || height < 0 || 
        width > dimWallWidth - 1 || height > dimWallHeight - 1 ){
        return 1; 
    }

    // imageArray tells us whether or not that pixel on the ground is black or white.
    // We treat 0 as shadow and 1 as light
    // If pixel is 0, we want a shadow so we should block (return 1), and vice versa.
    return 1-imageArray[int(width)*dimWallHeight + int(height)];
}