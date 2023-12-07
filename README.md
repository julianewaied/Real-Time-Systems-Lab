# Real Time Systems Lab
## About
### The Team
 Contributors: Julian Ewaied
 ###
 Lecturer: Dan Feldman
 ###
 Tutors: Bar, David and Arthur.
 ### About the Repository
 In this repository, we are building a 3D mapping of our lab's room using C++. We will be using a drone for this prupose with a rasberry pi microprocessor and a camera, to record 360 videos of the room, and extract feature points from it. For further information please contact us
 ## Installation
 ### Requirements
 you will need to have github desktop and visual studio installed on your machine
 ### How to Install
 1) Clone the repository to your machine by copying the link from github, then in Github desktop click ```file> clone repository ``` then paste the link and now you have the code on your machine.
 2) Build a C++ console app solution in visual studio, then right-click the header folder, click "Add existing file", navigate the repository and select all the header files in the include directory. Do the same for the source code.
 3) In the file containing the main function that is given by the visual studio, replace the code with the following code:
 ```cpp
 int Run();
#include <iostream>

int main()
{
	return Run();
}
 ```
 4) Finally, your solution is ready and runnable by visual studio.
