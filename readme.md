# Robot Code 2023

This repository is the main repository for tracking the code running on 2023's 
robot.

## SOR Util

Our robot code links against our team's shared library, SORUtil. 

See specific directories for directions on using functionality. Right now, we
have implemented the following features:

 * Motor controller agnostic abstractions over motors

## AdvantageKit

Our robot code also links against AdvantageKit provided by team 6328 Mechanical 
Advantage.

TODO: Guidelines for using this.

See https://github.com/Mechanical-Advantage/AdvantageKit/ for documentation and
source code.

## Building

To build and deploy the project, simply import into "WPILib 2023 VSCode" by
opening the folder the project is checked out into, and run the action 
"WPILib: Deploy robot code." 

## Project Guidelines

Except where specified, all code in this repository should follow the Google
Java style guide, available 
[here](https://google.github.io/styleguide/javaguide.html).

Note: Google's Style guide conflicts with WPILib examples and conventions;
prefer Google's Style where a conflict exists.

Development follows the 
["Git Feature Branch" workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow), 
as described by Atlassian. Potential changes should be made on branches specific
to a given feature, and proposed by creating a pull request into `main`.

SOR's template targets Java 17, which is required to use and deploy the robot code.
*Do not use features from standards above Java 17.*

### Branches

Pull requests into `main` should be signed off on by a mentor to ensure that
code standards are maintained.

Once the robot is created and a program has been tested for it, we will enter
"production" mode, where a branch `live` will be created from `main`. At this
point, the robot's code will always be kept to the version at `live`, except
for testing. For a change batch to be "promoted" to `live` from `main`, it
must undergo rigerous testing. 

Discipline in ensuring the code in `live` is thoroughly tested and is promoted
in batches ensures that there will always exist a version of the robot's code
that is known good and can be used in a match.

### Testing

Tests should be placed in the traditional Java directory (`src/test/java/`) 
corresponding to their location in the robot code. 

Tests are written with the JUnit 5 library.