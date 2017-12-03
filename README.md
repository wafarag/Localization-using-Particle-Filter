# Using Particle Filter for Accurate Vechile-Localization

<img src="Localization1.png" width="700" alt="Combined Image" />

# Overview
This repository contains all the code and data needed to implement a particle filter for accurate Localization of vehicles on a given map.

## Project Introduction
A vehicle is being autonomously driven and needs to know accurately where it is on a specific map. The details of map (landmarks) that includes the vehicle location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data are available.

In this project I have implemented a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation from the installed sensors (e.g. Lidar) on the the vehicle and control data from the vehicle gas/brake (velocity) pedals and the steering angle. 

## Running the Code
All the source code of the project is available [here](https://github.com/wafarag/Localization-using-Particle-Filter/tree/master/src).

This repository includes as well all the required [data](https://github.com/wafarag/Localization-using-Particle-Filter/tree/master/data) to run this code. 

