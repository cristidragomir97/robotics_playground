
* Containers - dockerfiles and compose for the current state of the blu robot
* Firmware - Arduino code for rosserial for sensor and motor boards
* Tools - tools used to profile the docker containers
    * docker-profile.py - Shows the currently used resources for all running docker containers (cpu, memory, disk and network usage)
    * list_libraries.py - Connects to the containers, uses lsof, and lists all runtime libraries, useful for copying only the essentials in multi-stage builds