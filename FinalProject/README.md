# Final Project - Give You a Gift

## How to set up

 1. git clone repo to home directory
 2. Run setup shell script
    ```
    cd ~/RobotSystems/armProject/setup.sh
    ```
 4. Wait for it to beep to reset everything
 3. Call the parrot service to start
    ```
    rosservice call /gifter/enter "{}"
    ```
 4. Set Running
    ```
    rosservice call /gifter/set_running "data: true"
    ```
 5. Set Target
    ```
    rosservice call /gifter/set_target "color:
    - 'red'
    - 'green'
    - 'blue'"
    ```



## Order of Operations

 - Look for cube
 - Grab cube
 - Lift cube
 - Look for Person
 - Wiggle at person
 - Drop cube!
 - Clacky clacky
 - Repeat from start

## Old

Parrot folder is old code from previous iteration attempt

