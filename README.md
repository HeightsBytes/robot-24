[![CI](https://github.com/HeightsBytes/robot-24/actions/workflows/build.yml/badge.svg)](https://github.com/HeightsBytes/robot-24/actions/workflows/build.yml)

# Code Rules
1. [Autonomous](#autonomous)    
2. [Libraries](#libraries)
3. [Coding Guidelines](#coding-guidelines)

## Autonomous
### Pathplanner
For our autonomous we use pathplanner, which can be found [here](https://github.com/mjansen4857/pathplanner).  
It allows us to remove ourselves from the heavy lifting that auto can be, and makes it extremely easy to create complex autonomous routines. 

### Pathplanner Naming Convention
When registering commands, they should be all lowercase and connected by underscores: example_command.  
When naming paths, they should be suffixed by the side of the field they are intended for. Example: loop_red.  
When naming autos, they should be suffixed with the \_auto in additon to the side of the field they are on. Example: shoot_2_and_leave_red_auto. 
Note: It is very likely that pathplanner will add path mirroring when the field is released, so these last few things can be ignored in that case. 

## Libraries
- [WPILib](https://docs.wpilib.org/en/stable/index.html)
- [REVLib](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information)
- [PathplannerLib](https://github.com/mjansen4857/pathplanner/wiki)
- [PhotonLib](https://docs.photonvision.org/en/latest/)
- [Phoenix(CTRE)Lib](https://pro.docs.ctr-electronics.com/en/latest/docs/yearly-changes/yearly-changelog.html)

## Coding Guidelines
### Pull Requests
While not required, it is highly encouraged that people use pull requests in order to ensure code quality.  

### Commits on main
It is highly discouraged to commit directly to main, people should create branches for features they intend to make. 