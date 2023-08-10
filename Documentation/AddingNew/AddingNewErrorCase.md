# Adding a new Error Case
Error cases should be specific and are structured to operate on a single joints data. 
## error_codes.h
1. In the ErrorCodes enum, create a name for your new error case. This name should be below the 
NO_ERROR case, and above the ERROR_CODE_LENGTH case. 

## error_types.h
1. Create a new error type class (using the other error types as a reference).
2. The class inherits from ErrorType and is required to have a check and handle function.
3. The check function should return True if your error has occured and false if it has not. 
Note: The check function only has access to the JointData class, and is ran for every joint.
If you're error case requires its own data, add that data to the JointData class (JointData.h)
4. The handle function decides what to do if your error has occured. Mostly disabling the motors.

## error_map.h
1. In the error_map, add a new key-value corresponding to your new error case. To do this,
use the following format:
    {YOUR_ERROR_CASE_NAME_FROM_PART_ONE, new YourErrorTypeNameFromPartTwo()},

## Done 
Now your error will be checked, using your check function by the run method in Joint.cpp for 
the hip, knee, and ankle. The error will be handled when it occurs by your handle 
function, and the error will be reported to the app. 