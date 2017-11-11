# OVS - Open Vehicle Simulator
## Intro
Purpose: Vehicle independent on-line vehicle simulator for evaluating feedback controllers

Creator: David Wong (davidwong.ucl@gmail.com)

Demo: [video](www.youtube.com)

License: 

## Road Map
0. matlab/octave prototype with good physics
1. cpp multi objects
2. multibody dynamics
3. environment interactions and collision

## Contents
* functions : 
* classes : 
* gui : simulators
* scripts : main sripts and test scripts
* config : model parameters, path definition, etc
* external: other's code

## Class Architecture
```
handle
 |
 +-- Model
 |    |
 |    +-- CornTFCarModel
 |    +-- CornODECarModel
 |    +-- YMCarModel
 |
 +-- Solver
 |    |  
 |    +-- AngVelFESolver
 |    +-- AngVelRK4Solver
 |    
 +-- Submodel
 |    |
 |    +-- TireModel
 |    +-- AirfoilModel
 |
 +-- Utilities
 |    |  
 |    +-- ???
 ```

## Notes
### Todo
* newton euler
* RK4
* sliding mode 
* lyanpunov
* ymd
* tiremodel


