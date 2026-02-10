# Inter Planet Museumvault - Automated Planning

This project is the implementation of the final evaluation assignment for the university course **Automated Planning**. It requires dealing with different planning solutions to gain hands-on experience with the theoretical topics covered during the lessons.

## Setup - Docker Image

Follow the steps below to build and run the project environment.

**Prerequisites**: 
Ensure you have **Docker Desktop** installed and running on your machine.

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

or 

(if first time do also ```chmod +x run_container.sh```):

```bash
./run_container.sh
```


**3. Activate Planutils**: 

navigate to the repository folder inside the container, then:

```bash
bash setup.sh
```

This will install prp planner (answer ```Y``` to the question about installing it) and it will activate the planutils environment (i.e. ```planutils activate```) for you.