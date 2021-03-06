RVO_Py_MAS
========

Python Implementation of Reciprocal Velocity Obstacle (RVO) for Multi-agent Systems

```
@ARTICLE{8361450,
  author={M. {Guo} and M. M. {Zavlanos}},
  journal={IEEE Transactions on Robotics}, 
  title={Multirobot Data Gathering Under Buffer Constraints and Intermittent Communication}, 
  year={2018},
  volume={34},
  number={4},
  pages={1082-1097},
  doi={10.1109/TRO.2018.2830370}}
```

-----
Description
-----
This package contains a **_plug-and-play_** Python package for collision-avoidance in multi-agent system, based on reciprocal velocity obstacles ([RVO](https://www.cs.unc.edu/~geom/RVO/icra2008.pdf)) and hybrid reciprocal velocity obstacles ([HRVO](https://www.cs.unc.edu/~geom/RVO/icra2008.pdf)).

It has _minimal impact_ on your control objective and requires _minimal integration_. 



<p align="center">  
  <img src="https://github.com/MengGuo/RVO_Py_MAS/blob/master/data/snapshots.png" width="800"/>
</p>

-----
Features
-----
* Takes a 2D workspace with _any number_ of non-overlaping circular or square obstacles
* _Any number_ of dynamic agents with non-zero volume.
* Allow the choice of VO, RVO, HRVO.
* **Direct plug-and-play** and **fully integrate-able  with your control objective**, i.e., the output velocity is a minimal modification of the desired velocity.

```python
from your_module import compute_desired_V, Update_V
from RVO import RVO_update

# your control objective here 
V_desired = compute_desired_V(X, control_objective, V_max)

# plug in the RVO controller from this package
V = RVO_update(X, V_desired, workspace_model)

# let the robot move
X = Update_X(X, V, step)
```

* Scalable and fast, see examples below. 
* See [example.py](https://github.com/MengGuo/RVO_Py_MAS/blob/master/example.py) for test run. [[Video1]](https://vimeo.com/185405407), [[Video2]](https://vimeo.com/185408368)


----
References 
----
* Papers on [RVO](https://www.cs.unc.edu/~geom/RVO/icra2008.pdf), [HRVO](https://www.cs.unc.edu/~geom/RVO/icra2008.pdf)
* There are [Python bindings](https://github.com/sybrenstuvel/Python-RVO2) of the C++ implementation from the algorithm developers. For my purposes, the formality is too _heavy_ to be integrated into my own project, so I have my own try.
* This package does _not_ depend on the [Clearpath geometric package](http://pcl.intel-research.net/publications/clearpath_sca2009.pdf) to compute velocity obstacles.


----
Discussion
----
* For **very** clustered workspace with a large number of robots, you may need to limit the `maximal velocity` and use very `small step size`.
* You may add additional constraints in `RVO_update` such as the change rate of `V`, the lower bound of `V`.
* When applying this module to experimental robot control, you may need to set the **step size** higher due to hardware constraints.
* In most practical experiments, this scheme should still work by limiting the _maximal velocity_.  

