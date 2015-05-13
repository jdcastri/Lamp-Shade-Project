#ifndef WALL_H
#define WALL_H 1

#include <iostream>
#include "../include/CompFab.h"

class Wall {
    public:
        int dimWallWidth, dimWallHeight;
        int* imageArray;
        CompFab::Vec3 normal, point;

        Wall(std::string imagePath, CompFab::Vec3 norm, int dimRoomX, int dimRoomY, int dimRoomZ);
        void setAxes(int dimRoomX, int dimRoomY, int dimRoomZ);
        void loadImage(std::string imagePath);
        CompFab::Vec3 rayWallIntersection(CompFab::Ray &ray);
        int shouldBlock(CompFab::Vec3 &meshVoxelPos, CompFab::Vec3 &lightSourcePos);

};

#endif