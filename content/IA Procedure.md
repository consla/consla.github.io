## Platform
### Hardware
The robot used for this experiment will be my FIRST Tech challenge team's competition robot. It is around 13.5x13.5 inches wide and a little under 11 inches tall. In order to perform localization (track its location) and therefore how much error off the desired path, it has a pair of odometry pods on the bottom. The drivetrain uses mecanum wheels, which are wheels with wheels angled at 45 degrees that allow holonomic movement.
![[mecanum vectors.excalidraw.dark.svg|500]]
> [!info] Holonomic movement
> A holonomic robot can move in any direction independent of heading (angle the robot is facing), which lets you do all sorts of crazy stuff like spinning while going in a straight line or orbiting in a circle. To be more formal, holonomic movement is when every part of the state (which in our case is its position and heading) can change independently of all other states.
> 
> However, due to a quirk of how mecanum wheels are made, the drivetrain will move significantly faster when going directly forwards and backwards than any other direction. In the GVF implementation, I will likely make it so that heading follows the direction of movement by default unless otherwise specified due to the speed boost.

> [!info] Odometry pods
> An odometry pod is a single unpowered omniwheel (wheel with rollers that allow it to freely slide sideways) attached to a rotational encoder that is sprung into the ground for traction. That encoder reads velocity, and that velocity is then read and integrated over time to achieve position. The smaller the wheels (more rotations per distance), the faster the refresh rate (smaller $dt$), and the better springing into the ground (more traction -> less slip), the more accurate the localization will be.
> 
> The reason why encoders aren't read off the motors that spin the mecanum wheels is because those wheels will drift: every time you run into a wall or another robot or anything, the wheels will keep spinning even though the robot isn't going anywhere and mess up the localization. Our current odometry system is accurate to $\pm 1$ inch over a $100$ inches or so. However, it is not well tuned nor well sprung and was recently driven around on sawdust (which introduces slip) in a wood shop for a demonstration for middle schoolers, so that accuracy can likely be improved.

![[bot_bottom.png]]

The robot will drive on a 12x12 foot field constructed of interlocked foam tiles, the same construction used for competition. If the place where I test ends up being too small I may reduce the size. The tiles will be coated with Staticide, but that should have a negligible effect on friction.
### Software
The computer on board the robot is essentially an Android phone with hardware to interface with motors and sensors slapped on top. Therefore, I will use Kotlin programming language, both because it is officially compatible with Android, and because it has many programming language constructs that make it nice to work in. Android is not ideal for robotics, but we are required to use it under the rules of the competition.

I will develop the GVF implementation ~~(which I have decided to call GOATed GVF, GG for short)~~ as a component of a library I am developing for my FTC team that contains various utilities. To measure how well the robot is tracking the path, the position of the robot, the error off the path in terms of direct distance, and the timestamp will be logged at roughly 60 $hz$ on the robot's SD card which will then be transferred over WiFi to my laptop.

The source code is and will continue to be publicly available online at the [West High Robotics Club Github repo](https://github.com/West-Robotics/FtcRobotController), licensed under the [BSD 3-Clause Clear License](https://spdx.org/licenses/BSD-3-Clause-Clear.html).

### Testing
The goals of the testing is to analyze how the robot performs path following with and without disturbances. There are 2 main aspects of each trial: the path and the obstacles. These will be changed across different tests to try and simulate a broad range of realistic situations in FTC competitions. The path is guaranteed to not change between

Each test will have 6 trials which will be as close to identical as possible. A test consists of having the robot autonomously follow a specified path with specified disturbances, like an obstacle placed in the path that the robot is guaranteed to collide with. The performance will be tracked by analyzing different aspects of how much error there is over time, like total error, presence of oscillations, and so on. Prior to each test, $k_{n}$ will be tuned without any obstacles present until the robot movement is critically damped (highest possible $k_{n}$ before oscillations occur) as we would do for competition, before introducing the obstacles for the actual tests.

#### Control variables
- Battery voltage
	- Charge multiple batteries and read their voltage with a voltmeter
- Mecanum wheel traction
	- Wipe down the wheels after every test to ensure that they are clean
- Tile sink (how much the robot sinks into the tile)
	- Perform all tests on the same floor
	- Soft carpet vs linoleum for example has a major effect on performance
- Tile friction
	- Don't let people step on the tiles with shoes and clean all the tiles before the first test
### Safety
There are no genuine safety concerns besides not putting your fingers between the gears of the robot servos which will hurt a lot if it moves. So I won't do that.