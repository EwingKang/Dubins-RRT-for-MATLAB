# Dubins-RRT-for-MATLAB
 
# About
RRT (Rapidly-Exploring Random Trees) using Dubins curve, with collision check in MATLAB
 
# Intro
RRT, the Rapidly-Exploring Random Trees is a ramdomized method of exploring within dimensions. This method can effectively generate a path to reach any point within certain limited steps due to its random characteristics. This method is proprosed by LaValle, Steven M. in October 1998, in his technical report to Computer Science Department of Iowa State University as "Rapidly-exploring random trees: A new tool for path planning" Today, multiple variation of RRT method is widely applied with path planning problem among UAVs for ground based, aerial, and marinetime vehicles.   
In RRT_Dubins.m, the paths connecting each points, or the "edges" to the "vertices" are replaced with Dubins curve. This is a very common practice when dealing with dynamic system. Specifically for non-holonomic system, such as cars, airplanes or ships. Replacements of Dubins curve means the branches growing out of a vertex is always smooth in the sense tengency. This makes the path generated more feassibly then the original straight line connection style if taking dynamics into consideration.  
 
 
# Algorithm
For the sake of simplicity, I will discuss the algorithm only with 2-D planes. The problem is, given a starting point and limited boundre, how do we reach everypoint within the area systematically? The method itself is very simple, only repeative iteration of are 4 steps.  
    1. generate a random point, i.e, a "vertex"   
    2. find the closest vertex from the existing list   
    3. connect the new vertex to the closest existing vertex, so called "edge"   
    4. append the newly generated vertex and edge to the known list   
As the iteration goes, it looks like a tree consists of edges is growing within the boundry and thus named so.   
 
# Program
There are currently 3 main programs, you can see the comments in each file for more detail 
* RRT.m   
   standard baseline RRT algorithm   
* RRT_Dubins.m   
   RRT with Dubins curve as edge   
* RRT_obstacles.m   
   RRT with obstacles and collision check   
 
# References 
* Steven M. LaValle "Rapidly-Exploring Random Trees: A New Tool for Path Planning" 1998, tech. rpt C.S.Dept, Iowa State U 
* Sertac Karaman and Emilio Frazzoli "Sampling-based algorithms for optimal motion planning",2011, The International Journal of Robotics Research  
* Yoshiaki Kuwata, Gaston A. Fiore, Justin Teo, Emilio Frazzoli, and Jonathan P. How, "Motion Planning for Urban Driving using RRT" 
* RRT wikipedia: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree 
* My Dubins repo: https://github.com/EwingKang/Dubins-Curve-For-MATLAB 
* collision check1: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect 
* collision check2: http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
   
# TODOS
* RRT+Dubins+collisioncheck
* tree growing animation
