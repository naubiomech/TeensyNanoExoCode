## Overview
Doxygen is a tool that turns source code into documentation. By specifing strict
commenting guidlines Doxygen is able to generate extensive HTML based documentaion
from source code. It is also able to generate call and include diagrams.  

## Suggested Development Tool
The Doxygen Documentation Generator extension on VSCode by Christoph Schlosser makes
generating Doxygen Comments a breeze (not sponsored). If you are not using that tool
then you must refer to the Doxygen specification to generate proper comments. 

## Generate Documentation
1. Install Doxygen [here](https://www.doxygen.nl/download.html), in the Sources and
Binaries section. Make sure that Doxygen is [added to PATH](https://helpdeskgeek.com/windows-10/add-windows-path-environment-variable/#:~:text=To%20add%20a%20new%20path%2C%20simply%20click%20on,it%20and%20then%20click%20on%20the%20Edit%20button.). My doxygen install 
was located at C:\Program Files\doxygen\bin
2. Install the GraphViz executable [here](https://www.graphviz.org/download/).
3. Go to the root directory of this repository (TeensyNanoExoCode) and run $doxygen
4. It will take several minutes to finish generating its output. 
5. To visualize the output open the html/index.html file in an explorer. 