# Inter Planet Museumvault - Automated Planning

This project is the implementation of the final evaluation assignment for the university course **Automated Planning**. It requires dealing with different planning solutions to gain hands-on experience with the theoretical topics covered during the lessons.

## Setup - Docker Image

Follow the steps below to build and run the project environment.

**Prerequisites**: 
Ensure you have **Docker Desktop** installed on your machine.

**1. Build the Image**:
Open the terminal inside this project folder and run the following build command:

```bash
docker build --rm --tag myplanutils .
```
*Note: This process may take a significant amount of time (up to 30-45 minutes).*

**2. Run the Image**: 
To run the container, execute this script:

```bash
docker run -v "/$(pwd):/computer" -it --privileged --rm myplanutils bash
```

**3. Activate Planutils**: 
To run the planutils script we need to active it:

```bash
planutils activate
```

## Run Planner - Test Example
1) Go inside the test example directory: 
   ```bash 
   cd /computer
   cd /text_example
   ```
2) Run Fast Downward Planner:
   ```bash 
   downward --alias lama-first test_domain.pddl test_problem.pddl
   ```