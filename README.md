# overconstrained-locomotion
* This is my Undergraduate thesis. This work investegated possible locomotion of an overconstrained leg.
## Bennett's four-bar linkage
This is an exapmle, or the source of our ispiration, of a Bennett's linkage.
<div  align="center">
<img src="figs/Bennett_a.png" width = 50% height = 50% alt="bennett linkage" align=center /> 
</div>

## Overconstrained leg  
The overconstrained leg is literally half of the four-bar linkage. The first two joints 1 and 2 are chosen to be actuation.  
<div  align="center">
<img src="figs/Bennett_b.png" width = 40% height = 40% alt="overconstrained leg" align=center /> 
</div>

## Controller design through templates composition
We first studied a monopod with overconstrained leg. Its frontal plane and sagittal plane view is shown below:
<div  align="center">
<img src="figs/vertical.png" width = 60% alt="overconstrained leg" align=center /> 
</div>

Controlling this monopod jumping in a sagittal plane requires consideration of its foot placement that is not directly underneath the body. $\delta$ here is the angle of projection from leg to YZ plane. The hopping controller is inspired by PennJerboa [<sup>1</sup>](#penn-jerboa) and some other works [<sup>2</sup>](#raibert) [<sup>3</sup>](#BHop) [<sup>4</sup>](#Analysis).  
## Webots experiments
![webots sim](figs/vertical_webots.png)  
## Results
Here are some initial results of the monopod hopping in the sagittal plane.

<div  align="center">
<img src="figs/orbit_webots.png" width = 50% height = 50% alt="phase space" align=center />  

phase space

<img src="figs/sagittal_data_50.png" width = 50% height = 50% alt="phase space" align=center />

sagittal plane trajectories
</div>

Some experiments are recorded in 'videos'.
## References
<div id="penn-jerboa"></div>

[1] [The Penn Jerboa: A Platform for Exploring Parallel Composition of Templates](https://repository.upenn.edu/ese_reports/16/)
<div id="raibert"></div>

[2] [Legged robot that balance](https://www.amazon.com/Legged-Robots-Balance-Artificial-Intelligence/dp/0262681196)
<div id="BHop"></div>

[3] [Reactive Planning and Control of Planar Spring–Mass Running on Rough Terrain](10.1109/TRO.2011.2178134)
<div id="Analysis"></div>

[4] [Analysis of a Simplified Hopping Robot](https://doi.org/10.1177/027836499101000601)