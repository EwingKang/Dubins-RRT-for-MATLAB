# Dubins-RRT-for-MATLAB
 
# About
RRT (Rapidly-Exploring Random Trees) using Dubins curve, with collision check in MATLAB  

(image: RRT_Dubins_obstacles.m with 100, 1000, 10000 iterations respectively)
<img src="https://github.com/EwingKang/Dubins-RRT-for-MATLAB/raw/master/test_results/one-hundrede-iterations.png" alt="100 iterations" width="300">
<img src="https://github.com/EwingKang/Dubins-RRT-for-MATLAB/raw/master/test_results/one-thousand-iterations.png" alt="100 iterations" width="300">
<img src="https://github.com/EwingKang/Dubins-RRT-for-MATLAB/raw/master/test_results/ten-thousand-iterations.png" alt="100 iterations" width="300">
 
# Intro
RRT, the Rapidly-Exploring Random Trees is a ramdomized method of exploring within dimensions. This method can effectively generate a path to reach any point within certain limited steps due to its random characteristics. This method is proprosed by LaValle, Steven M. in October 1998, in his technical report to Computer Science Department of Iowa State University as "Rapidly-exploring random trees: A new tool for path planning" Today, multiple variation of RRT method is widely applied with path planning problem among UAVs for ground based, aerial, and marinetime vehicles.   
In RRT_Dubins.m, the paths connecting each points, or the "edges" to the "vertices" are replaced with Dubins curve. This is a very common practice when dealing with dynamic system. Specifically for non-holonomic system, such as cars, airplanes or ships. Replacements of Dubins curve means the branches growing out of a vertex is always smooth in the sense tengency. This makes the path generated more feassibly then the original straight line connection style if taking dynamics into consideration.  
 
# Algorithm
For the sake of simplicity, I will discuss the algorithm only with 2-D planes. The problem is, given a starting point and limited boundary, how do we reach everypoint within the area systematically? The method itself is very simple, only repeative iteration of are 4 steps.  
   1. Generate a random point, i.e, a "vertex"   
   2. Find the closest vertex from the existing list (euclidean distance or dubins path distance).
   3. Create an "edge" connect the new vertex to the closest existing vertex.  
   4. Check if the newly generated vertex and edge has collision with obstacles or not. Go back to step1 if conflict.
   5. Append (add) the new vertex and edge to the known vertices and edges list.
   6. Start from step1
As the iteration goes, it looks like a tree consists of edges is growing within the boundary and thus named so.   
 
# Running the programs
There are 4 main programs, see the comments in each file for more detail 
* RRT.m   
   standard baseline RRT algorithm   
* RRT_Dubins.m   
   RRT with Dubins curve as edge   
* RRT_obstacles.m   
   RRT with obstacles and collision check   
* RRT_Dubins_obstacles.m  
   Final complete algorithm: RRT with Dubins curve and collision check
   
Note: All plotting related function have the filename starts with plot_xxxxx_xxxx.m.
 
# References 
* Steven M. LaValle "Rapidly-Exploring Random Trees: A New Tool for Path Planning" 1998, tech. rpt C.S.Dept, Iowa State U 
* Sertac Karaman and Emilio Frazzoli "Sampling-based algorithms for optimal motion planning",2011, The International Journal of Robotics Research  
* Yoshiaki Kuwata, Gaston A. Fiore, Justin Teo, Emilio Frazzoli, and Jonathan P. How, "Motion Planning for Urban Driving using RRT" 
* RRT wikipedia: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree 
* My Dubins repo: https://github.com/EwingKang/Dubins-Curve-For-MATLAB 
* line-line collision check1: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect 
* line-line collision check2: http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
* point-line collision check: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
* arc-line collision check1: http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
* arc-line collision check2: https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
* arc-line collision check3: https://math.stackexchange.com/questions/1316803/algorithm-to-find-a-line-segment-is-passing-through-a-circle-or-not
   
# License
Released under GPLv3 license  
Copyright (c) 2018 Ewing Kang  
  
Note: Dubins path generator is a MATLAB re-written from Andrew Walker's work, which was originally distributed under MIT license in C language.

# TODOS
* [x] RRT+Dubins+collisioncheck
* [ ] fast localsearch
* [ ] tree growing animation
