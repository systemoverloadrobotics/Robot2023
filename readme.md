# Robot Code 2023

This repository is used as a template for robot projects on the team. This 
allows us to implement libraries with core functionality to reuse year-to-year.
See specific directories for directions on using functionality. Right now, we
have implemented the following features:

 * Motor controller agnostic abstractions over motors

*Note: when copying this project to make a robot project, this readme should be
updated*

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
to a given feature, and proposed by creating a pull request into `master`.

SOR's template targets Java 17, which is required to use and deploy the robot code.
*Do not use features from standards above Java 17.*

### Branches

Pull requests into `master` should be signed off on by a mentor to ensure that
code standards are maintained.

Once the robot is created and a program has been tested for it, we will enter
"production" mode, where a branch `live` will be created from `master`. At this
point, the robot's code will always be kept to the version at `live`, except
for testing. For a change batch to be "promoted" to `live` from `master`, it
must undergo rigerous testing. 

Discipline in ensuring the code in `live` is thoroughly tested and is promoted
in batches ensures that there will always exist a version of the robot's code
that is known good and can be used in a match.

### Testing

Tests should be placed in the traditional Java directory (`src/test/java/`) 
corresponding to their location in the robot code. 

Tests are written with the JUnit 5 library.