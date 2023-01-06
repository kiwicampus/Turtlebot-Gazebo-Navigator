<!-- https://www.makeareadme.com/ -->

<!-- ---------------------------------------------------------------------- -->
# **Gazebo Turtlebot Simulator** (Final Project for Integration Engineer)

<!--
Badges: On some READMEs, you may see small images that convey metadata, such
as whether or not all the tests are passing for the project. You can use Shields
to add some to your README. Many services also have instructions for adding a
badge.
Visuals: Depending on what you are making, it can be a good idea to include
screenshots or even a video (you'll frequently see GIFs rather than actual videos)
Tools like ttygif can help, but check out Asciinema for a more sophisticated method.
-->
<!-- https://shields.io/ -->
![GitHub stars](https://img.shields.io/github/stars/kiwicampus/Turtlebot-Gazebo-Navigator?style=social)
![GitHub followers](https://img.shields.io/github/followers/kiwicampus?style=social)
![GitHub forks](https://img.shields.io/github/forks/kiwicampus/Turtlebot-Gazebo-Navigator?label=Fork&style=social)
![Twitter Follow](https://img.shields.io/twitter/follow/kiwibot?style=social)

<img src="https://user-images.githubusercontent.com/43115782/167684386-91cc219e-c499-4f5d-8895-2b292f887563.png" alt="kiwi_banner" width="1200">

![GitHub labels](https://img.shields.io/github/labels/kiwicampus/Turtlebot-Gazebo-Navigator/Work%20in%20Progess)
![GitHub repo size](https://img.shields.io/github/repo-size/kiwicampus/Turtlebot-Gazebo-Navigator?label=Repo%20Size)
![GitHub All Releases](https://img.shields.io/github/downloads/kiwicampus/Turtlebot-Gazebo-Navigator/total?label=Downloads)
![GitHub issues](https://img.shields.io/github/issues-raw/kiwicampus/Turtlebot-Gazebo-Navigator?label=Open%20Issues)
![GitHub pull requests](https://img.shields.io/github/issues-pr-raw/kiwicampus/Turtlebot-Gazebo-Navigator?label=Open%20Pull%20Request)
![GitHub language count](https://img.shields.io/github/languages/count/kiwicampus/Turtlebot-Gazebo-Navigator?label=Languages)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![GitHub release (latest by date)](https://img.shields.io/github/v/release/kiwicampus/Turtlebot-Gazebo-Navigator)
![GitHub contributors](https://img.shields.io/github/contributors/kiwicampus/Turtlebot-Gazebo-Navigator)
![GitHub last commit](https://img.shields.io/github/last-commit/kiwicampus/Turtlebot-Gazebo-Navigator)
![Made from](https://img.shields.io/badge/From-Colombia-Yellow)

### **Kiwibot**

Hi Human! 

[**Kiwibot**](https://www.kiwibot.com/) links on-demand customers with you. We move atoms from point A to B safely, efficiently and affordably. We're improving people's lives with the world's most affordable and technologically advanced delivery service for local commerce worldwide. Since our start  in 2017, *Kiwibot* has made over 90,000 deliveries and built over 500 robots. This has allowed us to be the number one robot delivery platform on earth. 

Since early stages **Kiwibot** has shown amazing ideas, strategies, and developments to propose a new way to perform last mile delivery. All of that was embodied in a single box, our first version. However, over the years, different versions have significantly improved our logistics, operations, design, and especially software, using not only pure robotics, but also artificial intelligence. At this stage, we are ready to perform astonishing new developments for our industry as a world-leading company in the last mile delivery with new versions of robots.

### **Kiwibot Technology**

Autonomous navigation is at the core of most robotics companies, however despite claims that you may have heard no robot in the outdoors delivery market is 100% autonomous, instead all of them need to fall back to human teleoperators in challenging situations that they cannot solve by their own. At Kiwibot's AI&Robotics team we are working hard to increase further the amount of time the robots can navigate on their own; for that we rely on a combination of custom made software and high quality open source libraries in ROS2's ecosystem (to which we often contribute) like [Nav2](https://navigation.ros.org/), [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html), [gazebo](https://gazebosim.org/home) among others. This is the codebase you will be working on when you become part of the team.

## **The Project's Context**

Nav2 is a very flexible and configurable system that sits at the core of our autonomy stack. It aims to provide autonomous navigation capabilities to mobile robots and comprises several modules (servers) with smaller tasks that can be combined together to achieve this goal or can be used independently to provide specific functionalities to the system (ex: localization, path planning, etc).

This modular architecture makes it really easy to swap the algorithmic component of the different modules, allowing developers to choose the most suited approach for their needs or making it easier to program their own functionalities if the defaults don't fit their needs. Since you will have to make heavy use of nav2 is highly recommended that you read [its documentation](https://navigation.ros.org/concepts/index.html) before getting started

You will probably use Nav2 in your everyday work at Kiwibot so consider this project as the beginning of your training.

The main idea of the project is to build a simple navigation system with Nav2 and a simulated Turtlebot3 on gazebo. You will start using Nav2’s low-level interfaces in order to understand how the system works, and then you will progressively scale up to use its high-level interfaces. In the end, your robot should be able to follow an arbitrary set of waypoints and perform actions when arriving to each of them. 

This project aims to assess your ability to develop new features in complex software systems using ROS2 and Nav2 as a case study. When we review your project we will assess how well understood the systems in nav2 that you used so be ready to answer some theoretical questions. 

The project will comprise three sections that will be independent of each other, so if you feel stuck on one of them feel free to jump to the next one. Each section will have some extra points that will allow your project to stand out. Additionally to the project’s functionality, which account for 80% of the total grade, we will perform a general evaluation of

- [10%] The way you structure your solution (nice OOP will be rewarded)
- [5%] How you write documentation
- [3%] The use you make of git
- [2%] The code style (use a linter)

Explain how the final deliverable should look like and what they are expected to present

## **Dependencies**

For running the project you will need:

1. A computer with Ubuntu 22 or 20. Though you can certainly use other ubuntu distributions or virtual machines on another OS we won't provide support if you are not using the above.
1. [docker-ce](https://docs.docker.com/install/)
2. [docker-compose](https://docs.docker.com/compose/install/)

As you may have inferred by the above dependencies the project runs on a docker container. We recommend using [VS Code](https://code.visualstudio.com/) as the main IDE for development. Install the [Remote development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extensions and follow the `running_dev_container` instructions to write and test code inside the docker container.

You can also use other IDEs but note that all the project dependencies are installed in the [docker image](.devcontainer/Dockerfile) so you may have a hard time setting all that up on a different environment (VS Code does everything for you). Also we expect to be able to run your solution in the same docker image in our computers.

The simulator has been tested on a computer having a core i3 processor and no GPU, so you shouldn’t run into issues if you don’t have a super pro computer. However, if you have a small processor we do recommend setting ubuntu to work in performance mode and keeping your computer plugged into the AC adapter at all times.

A recommendation is to use VS Code as the main IDE for development. Make sure you also have installed in your host:

## **General Guidelines**

Section 1: Low-level interaction (python, provide node skeleton)

Point 1 - Use the controller server to move the robot
The controller server’s input is a path and its output is the linear and angular velocity the robot should have to follow that path.
At this point, you will have to have the robot follow a short straight path using the controller server
Details about the specific TODOs and some conceptual questions,. Gifs on how things should look like


Point 2 - Use the planner server to move the robot to a waypoint
The planner server’s input is a goal and its output is a path. At this point you will have to use a planner server to calculate a path to a goal and then pass this path to a controller server (replan every second)
Details about the specific TODOs and some conceptual questions. Gifs on how things should look like

Point 3 - Use behavior server to make the robot spin on its goal
The behavior server holds different behaviors so it does not have a predefined input or output. It's an action server holder that runs certain routines when called so they depend on the specific  action. At this point, you will have to make the robot spin 180 degrees once it gets to its goal.
Details about the specific TODOs and some conceptual questions. Gifs on how things should look like


Section 2: High-level interaction (python or C++, C++ gives more points, a node from the ground up)
If you completed section 1 successfully you may be wondering if using these action servers by hand is the best way to create a complete navigation system. Nav2 maintainers thought it wasn’t and they created the bt navigator: a module that interacts with all these action servers through a user-programmable behavior tree. 

Point  1 - Move the robot to a waypoint using NavigateToPose
Navigate to pose is an action server that takes in a pose and runs the action servers we used in part 1 following the logic on the navigate_to_pose_w_replanning_and_recovery behavior tree. At this point, you will have to use this action server to move the robot to a waypoint.
Details about the specific TODOs and some conceptual questions. Navigation-commander like node, do something with the feedback like plotting, etc,. Gifs on how things should look like

Point  2 - Change the behavior tree to make the robot spin on its goal
Now you will have to achieve the same you did in point 3 part 1 but using the behavior tree. Change the xml file so the robot spins 180 degrees when it gets to its goal.
Details about the specific TODOs and some conceptual questions. Optional: use Groot to show the BT. Gifs on how things should look like


-

Point  3 - Use a different planner and controller, including the velocity smoother
To demonstrate the advantages of nav2 modular architecture you will have to re-run the navigation using the RPP controller and the smac_2d planner. Additionally, you will have to use the velocity smoother.
Details about the specific TODOs and some conceptual questions. Gifs on how things should look like


Section 3: Custom plugin (C++, provide skeleton or detailed tutorial)
Point  4 - Write a custom plugin (to define which)
In this last point you will have to write a plugin for the XX server. Link to tutorial

### **License**
 
This project is licensed under [Apache 2](LICENSE), and thus all contributions will be licensed as such
as per clause 5 of the Apache 2 License:
 
5. Submission of Contributions. Unless You explicitly state otherwise,
any Contribution intentionally submitted for inclusion in the Work
by You to the Licensor shall be under the terms and conditions of
this License, without any additional terms or conditions.
Notwithstanding the above, nothing herein shall supersede or modify
the terms of any separate license agreement you may have executed
with Licensor regarding such Contributions.

## **Kiwibot AI & Robotics Team**
 
 
Meet our amazing team in charge of designing, building, manufacturing and giving life to our most loved friend *the Kiwibot.*
 
<img src="https://user-images.githubusercontent.com/43115782/179125310-60a0e618-7113-4a96-9fa7-9c5f04f19d1a.png" alt="kiwi_ai_team" width="1200">
 
 
🤖 **AI & Robotics**: [John Betancourt ](https://www.linkedin.com/in/johnbetacode/) [AI&Robotics Team Lead - Robotics Engineer], [Marcela Gomez ](https://www.linkedin.com/in/marcela-gomez-cardona-7b2190161/) [QA and testing], [Davidson Daniel ](https://www.linkedin.com/in/dadaroce/) [Ai & Robotics Engineer], [Carlos Alvarez ](https://www.linkedin.com/in/calvarez92/)[Senior ML Engineer], [Pedro Gonzalez ](https://www.linkedin.com/in/pedro-alejandro-g-95655ba2/)[Ai & Robotics Engineer ], [Nicolas Rocha Pacheco ](https://www.linkedin.com/in/nicol%C3%A1s-rocha-pacheco/)[Ai & Robotics Engineer ], [Wilmer David Garzón Cáceres ](https://www.linkedin.com/in/wilmergarzon/)[Ai & Robotics Engineer], [Salomón Granada Ulloque ](https://www.linkedin.com/in/salomon-granada-ulloque/)[Ai & Robotics Engineer], [Alejandro Serna Escobar ](https://www.linkedin.com/in/alejandro-serna-escobar-a6b493115/)[Ai & Robotics Engineer], [Camilo Pinzon ](https://www.linkedin.com/in/capinzonq/)[Service Desk Engineer], [Alejandro Naranjo ](https://www.linkedin.com/in/alejandro-naranjo-z/)[Routing and Mapping Engineer] / **Old Members** (❤️ We couldn't have done it without you - may the force be with you ❤️): [Santiago Hincapie](https://github.com/shpotes) [ML Engineer], [Rafael Rincon](https://www.linkedin.com/in/rafaelivanrinconfonseca371b1515a/) [Ai & Robotics Engineer], [Camilo Alvis](https://www.linkedin.com/in/camiloalvis/), [David Cardozo](https://www.linkedin.com/in/davidcardozo/), [Juan Galvis](https://www.linkedin.com/in/jdgalviss/), [Robin Deuber](https://www.linkedin.com/in/robin-deuber/), [Juan Jurado](https://www.linkedin.com/in/juanfjuradop/), [Jason Oviedo](https://www.linkedin.com/in/jason-oviedo-46611914/), [Juan Rios], [Camila Rincones](https://www.linkedin.com/in/camila-rincones-casta%C3%B1eda/), [Juan Ramirez](https://www.linkedin.com/in/juan-ramirez-franco-530b80129/ ), [Mario Morales](https://www.linkedin.com/in/emetricz/), [Milad Noori](https://www.linkedin.com/in/milad-noori-2854b8191/), [Mauricio Reyes](https://www.linkedin.com/in/mauricio-reyes-hurtado/), [Cristian Garcia](https://www.linkedin.com/in/cgarciae/).
 
There is also a huge team out of the engineering area which gives a lot of work for our *Kiwibot*. Even if they are not listed here, they are the key to achieve our goals as a company:
 
**CEO**: [Felipe Chávez Cortés](https://www.linkedin.com/in/afchavez/) / **Design**: [Alejandro Otalora ](https://www.linkedin.com/in/alejandrootalora/)[Head designer], [Leonardo Correa](https://www.linkedin.com/in/leonardo-correa-853b30aa/)[Mechanical designer] / **Manufacture**: [Natalia Pinilla](https://www.linkedin.com/in/natalia-andrea-pinilla-rivera-5214ab91/)[Manufacture manager] / **Hardware**: [Andres Rengifo](https://www.linkedin.com/in/andresr8/)[Electronic designer] / **All Kiwi Family**: [here](https://www.kiwibot.com/about-us) .

<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Kiwibot Around The World**
 
(USA) Berkeley | (Asia) Taipei | (LATAM) Medellin | Around The World
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://user-images.githubusercontent.com/43115782/87437073-76bae880-c5b3-11ea-8edc-0ffb7b558137.png" alt="kiwibot_berkeley" width="500"/>    | <img src="https://user-images.githubusercontent.com/43115782/87437036-6c005380-c5b3-11ea-9de6-51b3fb6a33de.png" alt="kiwibot_taipei" width="500"/> | <img src="https://user-images.githubusercontent.com/43115782/87436986-5b4fdd80-c5b3-11ea-872e-9912f88cd66b.png" alt="kiwibot_medellin" width="500"/> | <img src="https://user-images.githubusercontent.com/43115782/87437115-833f4100-c5b3-11ea-9da9-6755f9095545.png" alt="kiwibot_streets" width="500"/>

(USA) San Jose | (USA) Denver | (USA) Los Angeles | (USA) Miami
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://user-images.githubusercontent.com/43115782/97742943-f4e3c200-1ab2-11eb-9778-72938a5ae3e2.png" alt="kiwibot_st-jose" width="500"/>    | <img src="https://user-images.githubusercontent.com/43115782/97742892-e4334c00-1ab2-11eb-9ec8-ee23f63d628a.png" alt="kiwibot_denver" width="500"/> | <img src="https://user-images.githubusercontent.com/43115782/140562710-f7c85f42-fc16-4478-9088-9d8b8ebdd9ec.png" alt="kiwibot_la" width="500"/> | <img src="https://user-images.githubusercontent.com/43115782/140562684-c163cbf6-22ca-4160-9017-6c1c0ea10965.png" alt="kiwibot_miami" width="500"/>

(Arab Emirates) Dubai | (Virtual) Minecraft | Coming soon! | Coming soon!
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://user-images.githubusercontent.com/43115782/140562869-2daa25bc-c767-4822-982c-ed49cd9f01d6.png" alt="kiwibot_dubai" width="500"/>    | <img src="https://user-images.githubusercontent.com/43115782/140562913-2f70be96-f587-4258-be65-c7a41171a151.png" alt="kiwibot_minecraft" width="500"/> | <img src="https://user-images.githubusercontent.com/43115782/140563168-9b8dc29a-3550-4627-9a22-7e8978c64d04.png" alt="kiwibot_minecraft" width="500"/> | <img src="https://user-images.githubusercontent.com/43115782/140563168-9b8dc29a-3550-4627-9a22-7e8978c64d04.png" alt="kiwibot_minecraft" width="500"/> 
<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Kiwibot In The News**
 
 
| | | | | |
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
Portfolio | Businessinsider | The New York Times | Techcrunch | Newscientist
[<img src="https://www.portafolio.co/files/article_multimedia/uploads/2020/04/16/5e98bba4b76d0.jpeg" width="150">](https://www.portafolio.co/negocios/empresas/colombia-hoy-rappi-pone-en-marcha-plan-piloto-para-entregas-con-robots-539959)| [<img src="https://i.insider.com/5eaadbd1cdfd48542c3e6a48?width=900&format=jpeg&auto=webp" width="150">](https://www.businessinsider.com/delivery-startup-using-robots-in-colombia-to-avoid-coronavirus-2020-4)| [<img src="https://static01.nyt.com/images/2019/11/08/business/07wheels-oak1/merlin_163524789_c8c7379d-6f33-4008-92c7-1c5adc3991aa-superJumbo.jpg?quality=90&auto=webp" width="150">](https://www.nytimes.com/2019/11/07/business/kiwibot-delivery-bots-drones.html)| [<img src="https://media.metrolatam.com/2019/08/26/kiwibot1-7ecb4d6a040f81f0af995e9c9064dfcf.jpg" width="150">](https://techcrunch.com/2019/04/25/kiwis-food-delivery-bots-are-rolling-out-to-12-new-colleges/?guccounter=1&guce_referrer=aHR0cHM6Ly93d3cua2l3aWJvdC5jb20v&guce_referrer_sig=AQAAAI2Kb9jFB6ouWToUhpiZszmts3-4fsGKzyxGs1naC3LNkv7Ra7ye5VMEkmx66Z_ZN16i-pax1clzWpcj4OQ4FcWFgSEs0h5yvp0URTcr_streMHeGdn6X5z06TDTkN0PjRHFClW8yuSPlvqb7gjSQ7yQxITVfr_rhgThkA9SsOKy) | [<img src="https://images.newscientist.com/wp-content/uploads/2017/12/07174953/kj9fw9.jpg?width=778" width="170">](https://www.newscientist.com/article/2155830-food-delivery-robots-are-teaching-themselves-how-to-cross-roads/)
 
<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Kiwibot's Media**
[<img src="https://img.youtube.com/vi/zwO4Pw6FNCU/0.jpg" width="200">](https://www.youtube.com/watch?v=zwO4Pw6FNCU)
[<img src="https://img.youtube.com/vi/iyO9TJHEQ7E/0.jpg" width="200">](https://www.youtube.com/watch?v=iyO9TJHEQ7E)
[<img src="https://img.youtube.com/vi/4l6janxFHyg/0.jpg" width="200">](https://www.youtube.com/watch?v=4l6janxFHyg)
[<img src="https://img.youtube.com/vi/0BKYZx42hwg/0.jpg" width="200">](https://www.youtube.com/watch?v=0BKYZx42hwg)
[<img src="https://img.youtube.com/vi/KZY0jfp6vmo/0.jpg" width="200">](https://www.youtube.com/watch?v=KZY0jfp6vmo)
[<img src="https://img.youtube.com/vi/w3znhvCiBxk/0.jpg" width="200">](https://www.youtube.com/watch?v=w3znhvCiBxk)
 
📷 **Instagram**: [kiwibot_us](https://www.instagram.com/kiwibot_us/), [kiwibot.taipei](https://www.instagram.com/kiwibot.taipei/), [kiwicampus](https://www.instagram.com/kiwicampus/)/
🤖 **Facebook**: [kiwicampus](https://www.facebook.com/kiwicampus/). Follow us and give us a star ⭐ if this project helped you or you like it!

<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Kiwibot's Projects Media**
 
 [<img src="https://img.youtube.com/vi/m7jZsa-QVMQ/0.jpg" width="200">](https://www.youtube.com/watch?v=m7jZsa-QVMQ)
[<img src="https://img.youtube.com/vi/c6agroNH1t0/0.jpg" width="200">](https://www.youtube.com/watch?v=c6agroNH1t0)
[<img src="https://img.youtube.com/vi/s5YpJLkszts/0.jpg" width="200">](https://www.youtube.com/watch?v=s5YpJLkszts)
[<img src="https://img.youtube.com/vi/szuZsLWioAQ/0.jpg" width="200">](https://www.youtube.com/watch?v=szuZsLWioAQ)
[<img src="https://img.youtube.com/vi/yGe7s1Azp9w/0.jpg" width="200">](https://www.youtube.com/watch?v=yGe7s1Azp9w)
[<img src="https://img.youtube.com/vi/zORfZK8v2TY/0.jpg" width="200">](https://www.youtube.com/watch?v=zORfZK8v2TY)
[<img src="https://img.youtube.com/vi/V9ZiZSh1MZM/0.jpg" width="200">](https://www.youtube.com/watch?v=V9ZiZSh1MZM)
[<img src="https://img.youtube.com/vi/Z4jQc3-psy8/0.jpg" width="200.com/vi/0U3lQ1u-Hd4/0.jpg" width="200">](https://www.youtube.com/watch?v=0U3lQ1u-Hd4)
[<img src="https://img.youtube.com/vi/VX4ywdUtaI4/0.jpg" width="200">](https://www.youtube.com/watch?v=VX4ywdUtaI4)
[<img src="https://img.youtube.com/vi/0BHpz8lCFfc/0.jpg" width="200">](https://www.youtube.com/watch?v=0BHpz8lCFfc)

General context about nav2
Nav2 is a software project that aims to provide autonomous navigation capabilities to mobile robots. It comprises several modules (servers) with smaller tasks that can be combined together to achieve this goal or can be used independently to provide specific functionalities to the system (ex: localization, path planning, etc).

This modular architecture also makes it really easy to swap the algorithmic component of the different modules, allowing developers to choose the most suited approach for their needs. Since you will have to make heavy use of nav2 is highly recommended that you read its documentation before getting started


