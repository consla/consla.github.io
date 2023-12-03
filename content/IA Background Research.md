## Introduction
In the FIRST Tech Challenge, a highschool competitive robotics competition, a very important part of well-performing robots is having good autonomous control of the drivetrain. The first 30 seconds of the 2 minute 30 second matches are completely autonomous and the robot must complete objectives without any driver input. Fast, accurate, and reliable path following is critical to scoring high amounts of points. The objective of this project is to implement both a path follower called a guiding vector field (GVF) which has a variety of benefits over traditional FTC path followers like Pure Pursuit, PID to point, or time-based motion profiled paths, and a path generator that enforces $C^{1}$ continuity (continuous velocity) and allows easy design of precise paths.
## GVF
[This paper](https://arxiv.org/pdf/1610.04391.pdf) describes what a GVF is, the advantages and disadvantages, and rigorous mathematical proofs. However, it is a very dense paper and the main concepts can be more simply described. Additionally, it leverages properties of using level set functions to define paths which ends up being somewhat difficult to do in implementation, and as the custom path generator will output a parametrically defined function for reasons that will be described [[#Path Generation|later]], some equations and methods will be modified to fit this version.

> [!question]- Why/what is a level set?
> The GVF paper uses level set functions which are essentially slices of a 3D function, where you set Z to be a certain value. For example, $x^{2}+y^{2}=c$ where $c=1$ would be the level set $1$ of a circle. One of the level sets is used as the desired path. It can be thought of as a topographic map, where $c$ is the "height" of the graph, so as $c$ increases the circle becomes both bigger and taller. Therefore this graph would produce an upside down cone if we set $c$ as the $z-$axis)
> 
> ![[IA Background Research Level Sets.excalidraw.dark.svg|center]]
%%[[IA Background Research 2023-12-02 23.00.41.excalidraw.md|🖋 Edit in Excalidraw]], and the [[IA Background Research 2023-12-02 23.00.41.excalidraw.light.svg|light exported image]]%%
> 
> Level sets are convenient for a GVF, because going up/down the gradient of the level sets will descend to the nearest point of the desired level set, without using any heuristics or approximations. However, these functions are difficult to generate when you want very precise pathing, so my implementation will not use it.

The GVF gets its name from generating a vector at each point on a coordinate plane that when followed will asymptotically converge the robot onto the desired path. Here are 2 neat visualizations I created, with one path that converges onto an ellipse and the other onto a parabola (sorry darkmode users).

![[gvf_ellipse.png|500]]

![[gvf_parabola.png|500]]

A GVF uses a combination of 2 vectors: the unit tangent vector of the nearest point on the path and the error vector pointing towards that nearest point.

![[IA Background Research GVF.excalidraw.dark.svg|center]]
%%[[IA Background Research 2023-12-02 16.14.03.excalidraw.md|🖋 Edit in Excalidraw]], and the [[IA Background Research 2023-12-02 16.14.03.excalidraw.light.svg|light exported image]]%%

$\hat{\tau}$ is the unit tangent vector of the path at the nearest point, $\vec{e}$ is the error function, $k_{n}\vec{e}$ is the weighted error vector, $\vec{v}$ is the resulting vector from adding $\hat{\tau}$ and $k_{n}\vec{e}$, and $\hat{m_{d}}$ is the unit vector of $\vec{v}$, in other words just the desired direction of motion. It is interesting and intuitive to note that for any point on the path, the positions of the robot for which is the nearest point lies along the direction given by the normal vector $\hat{n}$.

> [!note] Error function caveats
> Often, the error function is not just the nearest point to the path. In many cases, squared error or other transformations are desired because bigger errors will have a disproportionately larger influence on $\hat{m_{d}}$ than smaller errors. Which error function is used depends on the application.
> 
> Additionally, $k_{n}$ is just a way to blend the weights of $\hat{\tau}$ and $\vec{e}$. Bigger $k_{n}$ means $\vec{e}$ has a heavier weight and so the robot path will converge to the path quicker at the potential expense of sharper movements and oscillations, while smaller $k_{n}$ means $\hat{\tau}$ has a heavier weight and so that robot will move smoother and follow the relative movement of the path more, at the expense of slower path convergence. Selection of $k_{n}$ also depends on the application and is sometimes empirically tuned.

$\hat{m_{d}}$ is the only vector that the robot actually follows since GVF simply determines the direction in which the robot will travel. Velocity is independently controlled through other means, which will be touched on later.

The vector field can be described very succinctly like so:
$$
\vec{v}(x, y) = \hat{\tau}(x,y) + k_{n}\vec{e}(x,y)
$$
$$
\hat{m_{d}}(x, y) = \frac{\vec{v}(x, y)}{\|\vec{v}(x, y)\|}
$$

### Why/why not GVF?
Let's look at the disadvantages to other commonly used path followers and then see how GVFs both solve and introduce problems.
#### PID to point
This is simply a [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) on position that follows a series of points. The major disadvantage of this is that this does not allow the robot to follow smooth paths, but instead start and stop between each movement. In a game where every second counts, this is undesirable and being able to follow smooth paths greatly saves time.
#### Pure Pursuit
A semi-popular option, this also takes a series of points, but smooths the movement out by drawing lines between consecutive points, drawing a circle around the robot of a specified *lookahead* radius, and then travelling towards the intersection between the circle and the line. This technique has been used to great success in the past with FTC. However, it suffers from error from the specified path on curves which is just a part of the controller design, poor turning performance when dealing with various curvatures (only applicable to implementations with a fixed lookahead radius), and problematic special cases like behavior when the error is greater than the drawn circle, among others.
#### Time-based motion profiled paths
This is the current most popular option in FTC by far, being used by a very popular and successful autonomous movement library called [Roadrunner](https://rr.brott.dev/docs/v0-5/tour/introduction/). This system can be thought of as an extension of PID to point: instead of having a set of points that the robot travels to, the robot continually tracks the path using PID, with a reference point that moves along a parametrically defined function with acceleration and continuous velocity in accordance with a [trapezoidal motion profile](https://www.ctrlaltftc.com/advanced/motion-profiling). This solves many problems, with smooth movement and good path convergence.

However, it is time-based, which only performs well when the robot follows the expected time taken. The FTC environment is unpredictable, and events such as accidental robot or field collisions are rare but possible enough to matter. In situations like that, the robot using this system will either try to continue to follow the path which may make the situation worse, or it must detect when it goes off-course, cancel its trajectory, then find its way back. In addition, it is overall not very deterministic. Neither speed nor direction can be predicted due to being determined largely by time, and the robot's actual time taken will inevitably have large elements of randomness due to variations such as battery voltage, friction, and even how old the field floor is (worn surfaces).

#### GVF
GVFs are time-independent and speed-independent. That is, the controller does not depend on any aspect of time and is thus far more flexible, and speed can be externally controlled, which is extremely useful in fields such as aviation where the lift of fix-winged aircraft partially depends on aircraft speed, which makes unpredictable speed a big no-no. In the context of FTC, assuming your speed controller is also time-independent (which it wouldn't make sense not to be as that would destroy a major advantage of GVF), given soley the robot's position on the field, you can predict the exact input that will be given to your robot. This determinism is highly advantageous in multiple ways, including reliability, easier debugging, etc. GVF also has the advantages of following smooth paths and having good path convergence properties.

However, GVFs also have quite a major flaw, which is that by default it cannot handle a self-intersecting path, as the nearest point becomes ambiguous when near an intersection. The solution is to find a way to smoothly transition between 2 separate non-self-intersecting paths, the specifics of which are walked through later.
## Path Generation