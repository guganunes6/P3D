P3D

-> Open folder "MyRayTracer"

-> Open "MyRayTracer.sln"

-> Change the Solution Configuration to Release Mode

-> On main.cpp select which acceleration structure to use (on line 93):
  NONE: no acceleration structure;
  GRID_ACC: using grid acceleration structure;
  BVH_ACC: using BVH acceleration structure;

-> Change values (on lines 82-88):
  SPProot: to change samples per pixel;
  SPProot_shadow: to change samples on non antialiasing soft shadows;
  jittering: to turn on/off jittering;
  antiA: to turn on/off antialiasing;
  soft_shadows: to turn on/off soft shadows;
  fuzzy: to turn on/off fuzzy reflections;

-> To add new p3f scenes put them in the folder "MyRayTracer/P3D_Scenes"

-> Press "Local Windows Debugger" to start the program

-> If p3f_scene is turned off, write the name of the scene you want to test

-> ENJOY :3
