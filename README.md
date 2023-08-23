# sky360
Observational Citizen Science of Earths atmosphere and beyond.

Sky360.org aims to contribute to the research and understanding of UAP by providing a robotic system capable of autonomously observing the skies and capturing relevant data. 

## Bug Reports and Feature Requests

If you encounter a bug or have a feature request related to either the ROS2 backend or the OpenMCT frontend, you can contribute by opening an issue in the Sky360.org repository. To create an issue:

1. Go to the issue tracker of the repository.
2. Click on "New Issue".
3. Provide a descriptive title for the issue.
4. In the issue description, provide as much detail as possible, including steps to reproduce the bug or a clear explanation of the requested feature.
5. Submit the issue.

## What is needed for the station

@richard

## How to build the station

@richard or @christian

## How to install the software

TBD

## Contributing to Sky360.org
Please take a look at our [contributing.md](./contributing.md)

## System Architecture - WIP
The Sky360.org system follows a modular architecture, allowing different components to communicate and work together seamlessly. The primary components of the system include:

1. **Fisheye Camera**: The fisheye camera captures a wide field of view, enabling a comprehensive observation of the skies.
2. **Recorder**: Designed to record raw data from sensors as well as situational awarness. Currently in developement for the fisheye camera.
3. **OpenMCT**: OpenMCT (Open Mission Control Technologies) is utilized on the client side for visualizing and analyzing the collected data in a user-friendly manner.

The modular architecture of Sky360.org allows for flexibility and scalability. Additional components can be added in future versions to enhance the system's capabilities. The communication between the components is facilitated through ROS2, ensuring seamless integration and interoperability.

## Development team

## FAQ

Read [here](./FAQ.md)
