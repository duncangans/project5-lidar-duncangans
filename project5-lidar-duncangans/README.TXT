Duncan Gans
GIS Algorithms
November 21, 2017

lidarview.cpp
  This is the file that actually runs the lidar files and gets grids of both
the first return as well as an approximation of the ground. This was built off
of a elaborate framework designed by professor Toma. I have simply piggybacked
off of it and added the code necessary for the grids. The actual methods of
how the ground is discovered is outlined in greater detail in the actual code
as well as the paper in this file.

lidarviewGans.pdf
  This pdf file is the accompanying paper that describes the process of the 
lidarview file as well as the strenghts and weaknesses of the methods. 

fusa-parse-xyznrc.txt, lake-parse-xyznrc.txt, marse-parse-xyznrc.txt, 
sample-parse-xyznrc.txt, zurich-parse-xyznrc.txt
  These txt files are the ones used for the actual LidarPoints. The marse file
Does not work super well for the reasons described in the paper (an incorrect
lidar point). The function in lidarview.cpp is meant to be able to deal with the
other four lidar files relatively well. Obiously because the files are different
and each offer different problems, none of them is perfect, but all do a 
decent job of approximating the ground.

How to use the Files:
  To use the files, run ./lidarview <name of the txt file for rendering>. This
will initially create an image of all the points as well as the rendered ground.
The initial ground rendering will be done using a basic hill rendering method.
There are three buttons added by myself to change the functionality (also, it is
reccommended to click g, h, o, and v, to get rid of the points and better see
the grid). To toggle between hillshade methods, click 'n'. To switch between
the ground grid and the first return grid, click j and k. This shoud allow you
to see the full functionality of the grid. Also, it is recommended that you 
click W to fill the grid triangles as well as 2 to see the grid in 2d.

Problems, qualifications:
  Bugs arise in this function when the grid resolution is set to either extreme
in proportion with the txt files. Therefore the grid res is set to 200 at the
moment. The zInterval used is automatically scaled based on the size of the 
grid. What this means is that for the files in this folder, it should work 
relatively seamlessly. That said, if the grid resolution is changed, or there
are different files being used with vastly different dimensions, it will not 
work as well, and might crash. However, with the current code, I have not yet
run into it crashing at all. Obviously, my function isn't perfect. It will 
classify some non ground points as ground and some ground points as not ground. These problems are documented in the accompanying paper. However, on the whole
this function works well.



