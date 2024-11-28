# Project: AGV Communication Simulation

This project simulates communication between Automated Guided Vehicles (AGVs) using the RinSim framework. It allows the AGVs to exchange routing, location, and other relevant data to coordinate their movement in a shared environment. The system leverages the power of the `MessageContents` interface to create and send messages between agents in the simulation.

<b>A full report of the provided solution can be seen [here.](Report.pdf)</b>

## Overview

The core functionality of this project revolves around the exchange of messages that contain information about the sender's route, timestamp, Euclidean distance to the target, and other details. These messages are structured using the `Mssgs` class, which implements the `MessageContents` interface from RinSim. This class encapsulates all the necessary details to simulate AGV communication.

## Project Structure

The project contains the following main components:

- **`Mssgs.java`**: Implements the `MessageContents` interface, defining a message structure for communication between AGVs. It holds information like the sender's ID, route, timestamp, and distance to the target.
  
- **`MessageContents.java`**: The interface from the RinSim framework that is implemented by the `Mssgs` class to standardize message contents in the simulation.

- **Dependencies**: 
  - **RinSim**: The core simulation framework for modeling and simulating transportation and logistics operations.
  - **Guava**: A library used for handling `Optional` values to prevent null pointer exceptions and to handle missing data safely.

## Features

- **Routing Information**: AGVs can exchange routing information to ensure optimal paths are chosen in the environment.
  
- **Message Flexibility**: The `Mssgs` class allows for optional inclusion of various data points such as:
  - Handshake message content
  - Sender's route (list of points)
  - Timestamp of message transmission
  - Euclidean distance to the target
  
- **Safe Data Handling**: The use of `Optional` in the `Mssgs` class ensures that missing data is handled gracefully, without causing runtime errors due to null values.

## Setup and Installation

### Prerequisites

Before setting up the project, ensure you have the following installed:

- **Java** (version 8 or higher)
- **Maven** or **Gradle** (for dependency management)
- **RinSim** framework (dependencies managed by Maven or Gradle)
- **Guava** library (for `Optional`)

### Getting Started

1. **Clone the repository:**

   ```bash
   git clone https://github.com/your-username/agv-communication-simulation.git
   cd agv-communication-simulation
   ```

2. **Build the project using Maven (or Gradle):**

   If you're using Maven:

   ```bash
   mvn clean install
   ```

   Or if you're using Gradle:

   ```bash
   gradle build
   ```

3. **Run the Simulation:**

   Once the project is built, you can run the simulation. If you're using Maven:

   ```bash
   mvn exec:java
   ```

   Make sure to configure your simulation parameters as per your needs.

### Dependencies

Here are the core dependencies required by this project:

- **RinSim**: For creating and running the simulation model.
- **Guava**: For handling `Optional` types in a safe and null-free manner.

These can be added in your `pom.xml` or `build.gradle` file.

#### Example `pom.xml` (for Maven):

```xml
<dependencies>
    <dependency>
        <groupId>com.github.rinde</groupId>
        <artifactId>rinsim</artifactId>
        <version>2.0.0</version>
    </dependency>
    <dependency>
        <groupId>com.google.guava</groupId>
        <artifactId>guava</artifactId>
        <version>30.1-jre</version>
    </dependency>
</dependencies>
```

#### Example `build.gradle` (for Gradle):

```gradle
dependencies {
    implementation 'com.github.rinde:rinsim:2.0.0'
    implementation 'com.google.guava:guava:30.1-jre'
}
```

## Usage

1. **Sending Messages**: The `Mssgs` class is designed to hold and send various messages between agents. You can create a new `Mssgs` object, set data such as the route, sender's ID, and other parameters, and then send the message to other agents.

2. **Handling Messages**: On the receiving end, agents can extract the information from the `Mssgs` object using the provided getter methods such as `getSenderRoute()`, `getSenderID()`, `getTimeStamp()`, and `getEucDistance()`.

## Example Code Snippet

```java
// Creating a message
Mssgs message = new Mssgs();
message.setSenderID("AGV_001");
message.setTimeStamp(System.currentTimeMillis());
message.setEucDistance(500.0);
message.setSenderRoute(someRoute);

// Sending the message (this would be done through the simulation framework)
sendMessage(message);
```

## License

This project is licensed under the MIT License. See the LICENSE file for more details.

## Acknowledgments

- The [RinSim framework](https://github.com/rinde/rinsim) for providing the simulation environment.
- The [Guava library](https://github.com/google/guava) for providing the `Optional` class, which ensures safe handling of missing values.

---

This `README` outlines the basic structure of your project, provides instructions for setting it up, and describes the core functionality provided by the Java classes. You can modify and expand it based on the specific features and setup of your actual project.