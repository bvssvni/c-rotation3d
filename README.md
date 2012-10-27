c-rotate3d - Helper for 3D rotation around axis or with quaternion.  
BSD license.  
by Sven Nilsen, 2012  
http://www.cutoutpro.com  
Version: 0.000 in [angular degrees version notation](http://isprogrammingeasy.blogspot.no/2012/08/angular-degrees-versioning-notation.html)  

#Rotate3D

A helper for rotating around a center in 3D.
Supports rotation around an axis with an angle.
Supports [quaternions](http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation), which are good for rotation interpolation.
This can be used on space or color coordinates.

1. Set the rotation center.
2. Set the type of rotation.
3. Rotate a single point, rotate array of points, generate translate-rotate matrix.

Rotation of array supports n, step which can be used on formats xyz, xyzt, xxxx yyyy zzzz etc.

For more information, see 'rotate3d.h'.

##Compile unit tests

    gcc -o test-rotate3d *.c -Wall -O3
    
