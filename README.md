# Robotic Automation of Skills Labs
Bachelor thesis project on automated skills labs (GER)

## Context

In many hospitals, it has become common practice to automate processes and leave the logistics in parts to intelligent systems. However, this is not reflected in teaching, where a wide range of skills need to be learned. Especially in skills labs, where processes are practiced using simulations, a very low degree of automation will be found. This leads to simulations either requiring a lot of time and effort to set up or very few different scenarios being presented in order to minimize the accompanying time and effort.

However, most of the work involved in preparing simulation rooms can in practice be done by robots and computers nowadays. In particular, this includes setting up the simulation, managing and replenishing consumables, and preparing the rooms themselves. Automatic warehouse management is a widely researched field, as is the transport of goods in buildings and the construction of prefabricated components. Inventory management is also automated at companies by standard nowadays.

By reducing the workload of these tasks, instructors can be given more time to actually spend teaching. Automation also offers the opportunity to increase the quality of teaching. By reducing preparation time, more elaborate simulations can be run or a wider range of simulations can be added to the existing range.

The research task of this bachelor thesis was to develop a concept for a comprehensive automation system in skills labs. The system should be able to manage inventories and build simulations without the need for instructors to participate. It should be able to be used by instructors to simplify work in skills labs and to delegate tasks to the system.

## Adaptability

This is merely a prototype for a ROS network-based automation. While intended for the use in medical simulation labs, the same structure can be applied to all warehouse logistics projects involving mobile robotics plattforms.

Using automated SQL queries, the system is able to take user input and translate it into database requests and create a schedule to lend items out of the inventory. Existing sql databases using the same data structure can be used by simply dropping them into the respective folders. If none are present, the system will create new tables to start from scratch. 

All outwards connections are realized using ROS nodes. The only user connection is via the UI node, which can take console inputs for all functions or be called by an authorized external node to send commands to the system. This way, to-be-implemented user interface softwares have a standard gateway to access the system.

The navigation node connection uses unix time and a xy-coordinate system to give mission statements. As this is also realized using a ROS node, any common robot control software should be able to communicate with it via the standard ros interfaces.
