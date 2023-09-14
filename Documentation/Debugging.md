This document has some tips to help you through the software debugging process. Once you 
have made sure that you have a software issue (by testing the system with different hardware),
you should begin the software debugging proccess.  

# Overview
The debugging process can be divided into several parts. 
1. Isolation
2. Correction
3. Validation

# Isolation
The isolation step is where you determine what portion of code is causing your issue. For Arduino
we use Serial.print() in various code sections to determine where the error is. 

# Correction
Once you isolate the section of code causing your issue you must fix it! Ideally this can be done
by changing a line or two, but you may have to implement some new functionality. Be sure to develop
in a way that doesn't impact other users. 

# Validation
After you've made your changes, you must validate them. Test the code thoroughly on all supported
boards and/or systems before merging with main. Make sure your changes dont slow down the codebase
by profiling the runtime. 