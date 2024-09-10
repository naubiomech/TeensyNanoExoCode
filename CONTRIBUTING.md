Hello and welcome! This is an embedded codebase is used to help people move, and 
we're glad to have you contributing. 

## Overview
This codebase is intended to run on a Teensy 4.1 development board along with 
an Arduino Nano 33 BLE develepmont board. That's right, two microntrollers.
See the [Documentation](/Documentation/) for more details. The documentation also
has style guides and C++ development help. Your code should be commented in the 
[Doxygen](https://www.doxygen.nl/manual/docblocks.html) format. For help with this
and for help with generating the Doxygen output see [DOXY_HELP](/Documentation/DOXY_HELP.md).

## Ideal Development Workflow
This section will detail a step-by-step guide to making changes to the codebase. 
1. Create a new branch for your bug-fix or change with a descriptive name. See [Branch Management](#branch-management) for more
details.
2. Being sure to write code that will not impact the workflow of others, make your changes. Remember
to commit frequently and test as you go. See [debugging](/Documentation/Debugging.md) for more details. 
3. Validate that your changes function as expected on all applicable systems and supported board versions. 
Remember to profile your code, this ensures that you dont harm the codebases timing. See [profiling](/Documentation/Profiling.md) for more details. 
4. Once everything has been validated, merge your feature branch into main.

## Suggested Dev Tools
We use VSCode for much of our programming, but you can use whatever 
tool you prefer. VSCode is nice because is has a couple of extensions that make
development and Doxygen commenting easier. Install VSCode [here](https://code.visualstudio.com/Download). To install an extension, click on the Blocks icon on the 
left toolbar (On windows you can also press Ctrl+Shift+X). Then search for an extension 
that you would like. We use 'C/C++' (by Microsoft) and 'Doxygen Documentation Generator' 
by Christoph Schlosser. To use the automatic Doxygen comment generator simply go to
the line above the code you would like to comment and type /** and the hit enter. The 
extension should automatically add tags for parameters and return values (e.g. for a
function).

VSCode can be overwhelming at first but you can very quickly become productive. If you're
going to be using it to program a lot, I suggest taking the time to learn how to best
use it [here](https://code.visualstudio.com/learn/get-started/basics)

## Branch Management
### Branch Strategy
All changes to main require a pull request to be reviewed by another person. Please follow the Github Flow Branch Strategy, detailed [here](https://www.gitkraken.com/learn/git/best-practices/git-branch-strategy#github-flow-branch-strategy).
In this branch strategy the main branch is protected from frequent pushes and should at
all times be deployable. New features are added in what are called feature branches.
Feature branches should be created frequently for every small change, there should be no
direct changes to main! You can and should push changes to feature branches very
frequently. Once you have THOROUGHLY tested your feature branch, you may submit a pull
request to main. The feature branch should have a descriptive name. For example, if
you're working on implementing a new hip controller the branch name could be
'controllername-hip-controller'.

### Branch Deletion
Once a branch is not needed, meaning that it has been merged and validated, it could be deleted. This is up to the creator of the branch or someone
cleaning up old branches. Its best to keep a branch around in the even that its changes need to be debugged, but if it's properly validated there 
should be no need to debug the branch. 


## Libraries
If your code requires a new Arduino library be sure to add it to the Libraries folder. 
