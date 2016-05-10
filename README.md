# Humanoid Path Planner - Affordance module

Copyright 2016 LAAS-CNRS

Author: Anna Seppala

## Description
HPP - AFFORDANCE is a library that builds representations of a robot's environment in terms 
ofaffordances, which describe surrounding objects based on their application possibilities.
For instance, large vertical surfaces may be applied for leaning, whereas smaller horizontal
surface areas (e.g. stairs) allow for a robot to step on them.
This idea was presented in multiple papers by Kaiser et al. [(link)] (http://h2t.anthropomatik.kit.edu/english/21_459.php):

"Extracting Whole-Body Affordances from Multimodal Exploration" (IEEE/RAS 2014)

"Extraction of Whole-Body Affordances for Loco-Manipulation Tasks" (IJHR 2015)

"Validation of Whole-Body Loco-Manipulation Affordances for Pushability and Liftability" (IEEE/RAS 2015)

The HPP - AFFORDANCE module is loosely based on the methods of affordance extraction presented
above but analyses triangulated mesh files based on triangle normals.

This library is part of the software Humanoid Path Planner [(link)](http://projects.laas.fr/gepetto/index.php/Software/Hpp).

## Installation on ubuntu-14.04 64 bit with ros-indigo

To install HPP - AFFORDANCE, you will need to install one other package of the Humanoid Path Planner software with their respective dependecies. Please see the instructions below for the full installation of HPP - AFFORDANCE:


