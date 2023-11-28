This writeup is basically identical to the [page on GM0](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#deriving-mecanum-control-equations) but with diagrams. GM0 is very well written but I think this topic would benefit heavily from visual aides.
# Wheel Directions
Wheel directions are the biggest headache of implementing mecanum. The actual inverse kinematics involve some big maths but that's a pain so let's look at a more intuitive interpretation of how gamepad input relates with wheel directions.
![[mecanum 2023-11-22 01.14.39.excalidraw.svg]]
This is our standard mecanum drivetrain. The rollers are drawn like what the ground will see: the rollers should form an X if you're looking at the bot from above, but they form an O here cause we're looking at the bottom of the wheels from the top of the bot.
![[mecanum 2023-11-22 01.31.06.excalidraw.svg]]
Here's the drivetrain driving forwards. The black arrows indicate the wheel direction, and the green arrows point in the same direction as the force that each wheel actually exerts (the direction that the rollers don't spin). Each green force vector is split into its vertical and horizontal components. Orange dotted lines indicate components of the force that are cancelled out due to the other forces (there's an equal amount of force pointing both left and right), while green dotted lines indicated components of the force that are not cancelled out. Because we have 4 force components pointing forward, the overall movement of the driveshaft is also forwards. The combination of these different force vectors is the core of how we get mecanum to move in any direction we want.
## Coordinate system
We need to clearly define our coordinate system so that we don't get tripped up later.
Let's assume that in your code, all the motors have been properly reversed so that when power is set to $+1$, the wheels spin forward relative to the robot. Gamepad joystick coordinates are a little weird because pushing the joysticks forward results in a negative $y$, so let's also assume that the $y$-axis of the gamepad joystick input is reversed (multiplied by $-1$) in code, so that our joystick coordinate system look like:
![[mecanum 2023-11-22 08.15.43.excalidraw.svg]]
For simplicity, let's define our robot's coordinate system to match that of the joysticks, so:
![[mecanum 2023-11-22 08.26.15.excalidraw.svg]]
where $+y$ is forward.
> [!info]- Conventional robot coordinate systems
> In robotics and other related fields, forward is actually usually defined to be $+x$ and left is defined as $+y$ (in other words the axes is rotated $90^\circ$ counter-clockwise).
> ![[mecanum 2023-11-22 08.33.02.excalidraw.svg]]
> This is so that when $\theta$ (the angle from the positive $x$-axis) is $0$, the robot is pointing forward, which has some nice mathematical properties. However, for this writeup, we will use the same coordinate system as the joysticks to avoid any confusion.
## Derivations
Now, we have to figure out how [[]]