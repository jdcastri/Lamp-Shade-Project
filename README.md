# Lamp-Shade-Project
6.S079 Final Project

To run voxelization:<br>
cd build;<br>
<i>./voxelizer InputMeshFilename(OBJ) MeshDimension(int)</i><br>
Example: ./voxelizer input.obj 128
<br><br>
Voxelization will create an output text file (voxelizedX.txt) that holds the coordinates for all "on" voxels, where the
X in the name is the mesh dimension. In our example above, the output file would've been called "voxelized128.txt"<br>

To run lampshade (remove voxels):<br>
<i>./lampshade voxelizedX.txt OutputMeshFilename</i><br>
Example: ./lampshade voxelized128.txt output.obj
