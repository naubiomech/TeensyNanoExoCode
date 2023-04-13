# Libraries Added

## SPISlave_T4
- Git: https://github.com/tonton81/SPISlave_T4
- Commit: 28d8c1fd6082335d597483d45121d9db4c9cfc5c
- Modifications: 
    - SPISlave_T4.tpp 
        - line 18 changed to 

            ```static void lpspi4_slave_isr() {``` 

            to avoid multiple definition error.
## TSPISlave
- Git: https://github.com/tonton81/TSPISlave
- Commit: 492100f7f41a6d9d3888ed68b1e326a58f11f47a
- Modifications: None

## IniFile
- Git: https://github.com/stevemarple/IniFile
- Commit: 880edeac620f262b804fce500bfbbb1227c5cba9
- Modifications: None

## Adafruit_BluefruitLE_nRF51
- Git: https://github.com/adafruit/Adafruit_BluefruitLE_nRF51
- Commit: 16412c28c5d25eb2577c0d5bbac85f3c7dc7baae
- Modifications: 
    - Adafruit_BluefruitLE_SPI.cpp 
        - line 45 changed to:  
        
            ```SPISettings bluefruitSPI(1000000, MSBFIRST, SPI_MODE0);```  
            
            Due to teensy speed issue