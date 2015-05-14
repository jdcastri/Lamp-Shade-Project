# Lamp-Shade-Project
6.S079 Final Project

<b>To create executable</b><br>
mkdir build;<br>
cd build;<br>
cmake .. -G 'Unix Makefiles'<br>
ccmake ..<br>
<i>Under CMAKE_BUILD_TYPE enter 'DEBUG'</i><br>
<i>Press 'c', then 'g'</i><br>
make<br>

<b>To run voxelization:</b><br>
cd build;<br>
<i>./voxelizer InputMeshFilename(OBJ) MeshDimension(int)</i><br>
Example: ./voxelizer input.obj 128
<br><br>
Voxelization will create an output text file (voxelizedX.txt) that holds the coordinates for all "on" voxels, where the
X in the name is the mesh dimension. In our example above, the output file would've been called "voxelized128.txt"<br>

<b>To run lampshade (remove voxels):</b><br>
<i>./lampshade voxelizedX.txt OutputMeshFilename</i><br>
Example: ./lampshade voxelized128.txt output.obj

To change input images, inclusion of walls, or room dimensions: edit main() in /lamp_source/lampshade.cpp<br>
