# README
How to use the different packages to run the robot.

As of September 2020, our Trash-picking Turtlebot can autonomously navigate around the building, draw a map, detect & pickup water bottle, then drop into trashcan. See the videos here: https://drive.google.com/drive/folders/1VAmSW8Z5EUJZb4Y-RXb9zj3yCQSbMGp9

It requires these packages:
- Object tracking (https://github.com/Cornell-Tech-Turtlebot/object_tracking): Detect different objects including water bottle & trashcan. Localize positions of those objects on the map. Drive the robot toward those objects.
- Bottle manipulator (https://github.com/Cornell-Tech-Turtlebot/bottle_manipulator): Control the manipulator to pickup & drop off the water bottle.
- Patrol (https://github.com/Cornell-Tech-Turtlebot/patrol): Randomly patrol around the building.
- Orchestrator (https://github.com/Cornell-Tech-Turtlebot/orchestrator): Orchestrate other packages. So for example, when `object_tracking` successfully drive the robot to approach the bottle, `orcheschator` will tell `bottle_manipulator` to start picking the bottle up.


