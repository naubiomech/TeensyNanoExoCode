Profiling is used to determine how long it takes for the computer to run your code. 
Typically more advanced software is used to automatically determine how long
each section of code takes to run, but Arduino does not provide any profiler software. 

To profile a section of code call micros() above the section of code and micros() below 
the section of code. Then Serial.print() the difference. ie:
        float start_time = micros();
        // The section of code
        float end_time = micros();
        Serial.print("Elapsed time in microseconds: ");
        Serail.println(end_time - start_time);