RVO_Py_MAS
========

Python Implementation of Reciprocal Velocity Obstacle for Multi-agent Systems

-----
Description
-----
this package contains a _plug-and-play_ Python package for collision-avoidance in multi-agent system, based on reciprocal velocity obstacles. 

-----
Features
-----
* Takes a 2D workspace with any number of non-overlaping circular or square obstacles
* Any number of dynamic agents with non-zero volume.  
* Direct plug-and-play and fully integrate-able  with your **actual** control objective, i.e., the output velocity is a minimal modification of the decried velocity.  
* Scalable and fast.
* Use Pygame for easy visualization.

<p align="center">  
  <img src="https://github.com/MengGuo/Roadmap_Poly2tri/blob/master/data/example.png" width="800"/>
</p>

----
References 
----
* Paper on [RVO](http://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf)
* There are [Python bindings](https://github.com/sybrenstuvel/Python-RVO2) of the C++ implementation from the algorithm developers. For my purposes, the formality is too _heavy_ to be integrated into my own project, so I have my own try.
* This package does not depend on the [Clearpath geometric package](http://pcl.intel-research.net/publications/clearpath_sca2009.pdf) to compute velocity obstacles.