

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/NiyoushaRahimi/UW-MLP">
  </a>

  <h3 align="center">Machine Learning based Perception for Optimal Motion Planning and Control</h3>

  <p align="center">
    Niyousha Rahimi, RAIN Lab University of Washington
    <br />
    <a href="#usage">View Demo</a>
    ·
    <a href="https://github.com/NiyoushaRahimi/UW-MLP/issues">Report Bug</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      </li>
      <li>
      <a href="#Requirements">Requirements</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started-with-unreal-engin">Getting Started with Unreal Engin</a></li>
       <ul>
        <li><a href="#Building-initial-map">Building the initial map</a></li>
       </ul>
    <li><a href="#main-project">Main Project</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

In this work, a Machine Learning based perception module is developed using Masked-RCNN (and Bayesian Neural Networks) with RGB-D images to estimate the position of objects in an Unreal Engin environment. Two methods are offered for autonomous navigation:

* Sampling based approaches such as RRT^* and Astar 
* stochastic optimal control: Successive convexification for path planning.

It should be mentioned that some parts of the project is still under development.

## Requirements

Main requirements are as follows:
* [Unreal Engin 4.25.3](https://www.unrealengine.com/en-US/download)
* [Airsim](https://github.com/microsoft/AirSim)
* Tensorflow-gpu 1.13 (could work with v1.15)
* Keras 2.1.6



<!-- GETTING STARTED -->
## Getting Started with Unreal Engin

Please download and install unreal engin. 
I have created an unreal engine environment of an airport. This environment can be downloaded from here: 
* [Airport environment](https://drive.google.com/file/d/1zUhz1Me5F2KKPsuPvBpftNABcb4D2hnI/view?usp=sharing)

Make sure to load AirportShowcase and hit play before running any code.

![AirportShowcase](Images/Figure-1.png)

### Building the initial map

There are three frame of references we need to consider:
1. Unreal-engin coordinate frame
2. The moving vehicle coordinate frame (the origin of the airsim's coordinate frame is placed at the position of camera when the simulation was started)
3. The map's coordinate frame

The following figure demonstrates these coordinate frames and their origin:


![AirportShowcase](Images/Figure-2.png)


Code is provided in map.py for building the initial occupancy map.


![Initial occupancy map](Images/Figure_5.png)
<img src="Images/Figure_5.png" width="150" height="150">

## Main project

The main project is carried out in car-sim.py.
Given the inirtial occupancy map, RRT^* is used to draw 



<!-- USAGE EXAMPLES -->
## Usage




<!-- ROADMAP -->
## Roadmap







<!-- CONTACT -->
## Contact

Niyousha Rahimi - nrahimi@uw.edu

RAIN Lab, University of Washington







<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/newsha-rahimi/
[product-screenshot]: images/screenshot.png
