# Contribute to Sky360
We appreciate your interest in contributing to the development of Sky360.org! By contributing, you can help improve the system, add new features, and make it more robust in observing and detecting UAP. This guide outlines the steps to contribute effectively.

## Software Guidelines and Best Practices
* Follow the existing code style and guidelines used in the repository for both the ROS2 backend and the OpenMCT frontend.
* Write clear and concise commit messages and pull request descriptions.
* Provide appropriate documentation and tests for new fPeatures and bug fixes in both the ROS2 backend and the OpenMCT frontend.
* Be respectful and considerate in your interactions with other contributors.

--- 
*Note: This contribution guide provides a general overview. Please adapt it to match the specific contribution process and guidelines of your Sky360.org repository, taking into account the ROS2 backend and OpenMCT frontend contributions.

## Guidelines for Commit Messages
When making commits to the Sky360.org project, please follow these guidelines for writing commit messages:

- Begin the commit message with a capitalized verb in the present tense (e.g., "Add," "Fix," "Update," "Refactor").
- Keep the commit message concise and descriptive, summarizing the purpose of the commit.
- Use imperative mood (e.g., "Add new feature" instead of "Added new feature").
- Separate the subject from the body of the message with a blank line.
- In the body, provide more detailed information if necessary, such as explaining the rationale behind the change.

Example commit message:

```bash
Add new ROS2 feature: XYZ

This commit introduces a new ROS2 feature to improve the UAP detection accuracy. The XYZ module is responsible for performing advanced object tracking algorithms based on machine learning techniques. This feature enhances the system's ability to identify and track UAP effectively.
  
```

## Guidelines for Pull Requests

When opening a pull request (PR) for the Sky360.org project, please adhere to the following guidelines:

- Provide a clear and concise title for the PR, summarizing the changes made.
- Include a detailed description of the purpose and scope of the PR.
- Reference any relevant issues or feature requests in the description.
- Ensure that the PR includes all necessary changes, including code updates, documentation modifications, and tests, if applicable.
- Respond to any feedback or comments promptly and make the necessary updates accordingly.

Example pull request description:

```bash
Feature: Implement Real-Time Data Visualization in OpenMCT

This PR adds a real-time data visualization feature to the OpenMCT frontend. It includes a new widget that displays live telemetry data from the Sky360 robotic system. Users can now monitor the detected objects and their attributes in real time, enhancing the situational awareness during UAP observations.

Closes #42

```

## Contact

If you have any questions or need further assistance, feel free to reach out through the issue tracker.

We appreciate your contributions and thank you for your support in advancing the Sky360.org project!

# Dependencies

- docker 
- Visual Studio Code 
- dev containers extension
  https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers

## Develop 

1. Fork the repository on GitHub.
2. Clone your forked repository to your local machine.

```bash
git clone https://github.com/sky360-repository/sky360.git

cd sky360
```

## Guidelines

- ROS2 (backend)
  ```bash
  cd src/ros2
  code .
  ```
  Then open the folder in dev. container. (Dev Containers: Open Folder in Container... command from the Command Palette or quick actions Status bar item)

- OpenMCT (frontend)

## ROS2 (backend) 

### create a new ROS2 node. 

C++

## OpenMCT (frontend)