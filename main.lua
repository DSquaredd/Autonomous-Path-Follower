function sysCall_init()
    -- objects representing mobile robot
    objHandle=sim.getObjectHandle(sim.handle_self)
    result,robotName=sim.getObjectName(objHandle)
    lineTracerBase=sim.getObjectHandle("LineTracerBase")
    leftSensor=sim.getObjectHandle("LeftSensor")
    rightSensor=sim.getObjectHandle("RightSensor")
    middleSensor=sim.getObjectHandle("MiddleSensor")
    
    -- mechanical degrees of freedom
    leftJointDynamic=sim.getObjectHandle("DynamicLeftJoint")
    rightJointDynamic=sim.getObjectHandle("DynamicRightJoint")
    
    -- mobile robot specs
    nominalLinearVelocity=0.29
    wheelRadius=0.027
    interWheelDistance=0.119 -- length of wheel axles (center of robot to wheel)
    arcLength = interWheelDistance * 2.9 -- 2.9 radians == 166 degrees
    timeToTurn90Degrees = arcLength / nominalLinearVelocity -- time to turn x degrees
    timeToTravelStraight = 0.11 -- used for searching straight
    
    s = sim.getObjectSizeFactor(objHandle)
    linearVelocityLeft = nominalLinearVelocity
    linearVelocityRight = nominalLinearVelocity
    sim.setJointTargetVelocity(rightJointDynamic,linearVelocityRight/(s*wheelRadius))
    sim.setJointTargetVelocity(leftJointDynamic,linearVelocityLeft/(s*wheelRadius))
    
    -- used to keep track of total primary search attempts before using secondary search
    full_search_attempts = 0; 

    -- variables to keep track of laps and robot direction
    initialPosition = sim.getObjectPosition(objHandle,-1)
    turningAround = 0
    turnedAround = 0
    totalLaps = 0
    
    -- immediate sensor readings
    left_sensor = sim.readVisionSensor(leftSensor)
    right_sensor = sim.readVisionSensor(rightSensor)
    middle_sensor = sim.readVisionSensor(middleSensor)
    
    -- previous sensor readings
    previous_left_sensor_prev = 0
    previous_right_sensor_prev = 0
    previous_middle_sensor_prev = 0
    
    -- previous previous sensor readings
    previous_left_sensor_prev_prev = 0
    previous_right_sensor_prev_prev = 0
    previous_middle_sensor_prev_prev = 0
    
    -- variables required for secondary path finding method
    search_complete = 0
    search_end_time = 0
    search_complete_xy = 0
    straight_back_time = 0
    straight_search_slope = 0
    straight_search_adjust_time = 0
    slope_adjust = 0
    slope_adjust_time = 0    

    -- variables required for primary path finding method (rotating left and right)
    searching = 0;
    searched_left = 0;
    searching_left = 0;
    searched_right = 0;
    searching_right = 0;
    searched_straight = 0;
    searching_straight = 0;
    turn_left_undo = 0;
    turn_right_undo = 0;
end

function sysCall_actuation()    

    -- calculate euclidian distance of current position and initial position
    currentPosition = sim.getObjectPosition(objHandle,-1)   
    x2 = currentPosition[1]
    y2 = currentPosition[2]
    distanceD = math.sqrt((x2 - initialPosition[1])^2 + (y2 - initialPosition[2])^2)
    
    -- check if one lap has been completed
    if(totalLaps < 1 and turnedAround == 0 and turningAround == 0 and 
        sim.getSimulationTime() > 5 and distanceD < 2 and 
        (initialPosition[2] - currentPosition[2]) > 1) then
        turningAround = 1
        previousTime = sim.getSimulationTime()
        sim.setJointTargetVelocity(rightJointDynamic, 0) 
        sim.setJointTargetVelocity(leftJointDynamic, 0)
    end
    
    -- rotate in place a few degrees without reacting to sensor readings
    if (totalLaps < 1 and turningAround == 1 and 
        (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees / 4) then
        -- turn left
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius/4) 
        sim.setJointTargetVelocity(leftJointDynamic, -nominalLinearVelocity / wheelRadius/4)
        turnedAround = 1
    -- continue rotating in place until one of the sensors detects the track
    elseif (totalLaps < 1 and turningAround == 1 and 
            ((sim.getSimulationTime() - previousTime) < timeToTurn90Degrees or 
            (middleResult == 1 and leftResult == 1 and rightResult == 1))) then
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius/8) 
        sim.setJointTargetVelocity(leftJointDynamic, -nominalLinearVelocity / wheelRadius/8)
        turnedAround = 1
    -- increment lap counter once the robot has turned around
    elseif(totalLaps < 1 and turningAround == 0 and turnedAround == 1) then
        turnedAround = 0
        totalLaps = totalLaps + 1
    -- follow the path 
    else
        followPath()
    end
end

function followPath()
    -- Middle sensor detects track. Go forward
    if (middle_sensor==0) then
        resetSearchVariables()
        resetSecondarySearchVariables()
        linearVelocityLeft=nominalLinearVelocity
        linearVelocityRight=nominalLinearVelocity
        sim.setJointTargetVelocity(leftJointDynamic,linearVelocityLeft/(wheelRadius))
        sim.setJointTargetVelocity(rightJointDynamic,linearVelocityRight/(wheelRadius))
    -- Only left sensor detects track. Go right
    elseif (left_sensor==0 and middle_sensor==1 and right_sensor==1) then
        resetSearchVariables()
        resetSecondarySearchVariables() 
        linearVelocityLeft=nominalLinearVelocity/10
        linearVelocityRight=nominalLinearVelocity
        sim.setJointTargetVelocity(leftJointDynamic,linearVelocityLeft/(wheelRadius))
        sim.setJointTargetVelocity(rightJointDynamic,linearVelocityRight/(wheelRadius))    
    -- Only right sensor detects track. Go right
    elseif (left_sensor==1 and middle_sensor==1 and right_sensor==0) then
        resetSearchVariables()
        resetSecondarySearchVariables()
        linearVelocityRight=nominalLinearVelocity/10
        linearVelocityLeft=nominalLinearVelocity
        sim.setJointTargetVelocity(leftJointDynamic,linearVelocityLeft/(wheelRadius))
        sim.setJointTargetVelocity(rightJointDynamic,linearVelocityRight/(wheelRadius))
    -- sensors do not detect track
    elseif(full_search_attempts < 2 and left_sensor==1 and right_sensor==1 and middle_sensor==1) then
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        sim.setJointTargetVelocity(rightJointDynamic, 0)
            
        -- search left first
        if((previous_left_sensor_prev_prev == 0 and 
            previous_middle_sensor_prev_prev == 0 and previous_right_sensor_prev_prev == 0) and
            (previous_left_sensor_prev == 0 and 
            previous_middle_sensor_prev == 0 and previous_right_sensor_prev == 1)) then
            primarySearch_CheckLeftFirst()
        elseif((previous_left_sensor_prev_prev == 0 and 
            previous_middle_sensor_prev_prev == 0 and previous_right_sensor_prev_prev == 1) and
            (previous_left_sensor_prev == 0 and 
            previous_middle_sensor_prev == 1 and previous_right_sensor_prev == 1)) then
            primarySearch_CheckLeftFirst()  
        elseif((previous_left_sensor_prev_prev == 0 and 
            previous_middle_sensor_prev_prev == 1 and previous_right_sensor_prev_prev == 1) and
            (previous_left_sensor_prev == 0 and 
            previous_middle_sensor_prev == 0 and previous_right_sensor_prev == 1)) then
            primarySearch_CheckLeftFirst()
        -- search right first
        else 
            primarySearch_CheckRightFirst()
        end
    elseif(full_search_attempts >= 2 and search_end_time == 0) then
        -- go straight after searching a few times
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius)
        sim.setJointTargetVelocity(leftJointDynamic, nominalLinearVelocity / wheelRadius)
        search_complete = 1
        search_end_time = sim.getSimulationTime()
    elseif((sim.getSimulationTime() - search_end_time) < 2) then
        -- do nothing
    elseif(full_search_attempts >= 2 and search_complete == 1 and 
        (sim.getSimulationTime() - search_end_time) >= 2 and straight_back_time == 0)then
        -- brake
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        -- if vehicle is at a (presumed) full stop
        if((sim.getSimulationTime() - search_end_time) >= 2.25) then -- gives time to stop vehicle
            sim.setJointTargetVelocity(rightJointDynamic, -nominalLinearVelocity / wheelRadius)
            sim.setJointTargetVelocity(leftJointDynamic, -nominalLinearVelocity / wheelRadius)
            search_complete_xy = sim.getObjectPosition(objHandle, -1)
            straight_back_time = sim.getSimulationTime()
        end

    -- Busy wait until robot has moved backwards for x second        
    elseif((sim.getSimulationTime() - straight_back_time) < 2.0) then
        -- do nothing
    elseif((sim.getSimulationTime() - straight_back_time) >= 2 and slope_adjust == 0) then
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        if(sim.getSimulationTime() - straight_back_time >= 2.25) then
            -- find the slope of the line from the starting to ending point of the straight line search
            currentPos = sim.getObjectPosition(objHandle,-1)
            straight_search_slope = (currentPos[2] - search_complete_xy[2])/(currentPos[1] - search_complete_xy[1])
            search_complete = 0
            slope_adjust = 1
            slope_adjust_time = sim.getSimulationTime()
        end
    -- rotate the robot for a few seconds in a direction dependent on the slope 
    elseif(slope_adjust == 1 and (sim.getSimulationTime() - slope_adjust_time) < 0.5) then
        if(straight_search_slope > 0) then
            -- slope is greater than 0 so move robot upward
            sim.setJointTargetVelocity(rightJointDynamic, 0)
            sim.setJointTargetVelocity(leftJointDynamic, nominalLinearVelocity / wheelRadius / 10)
        else
            -- slope is less than 0 so move robot downward
            sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius / 10)  
            sim.setJointTargetVelocity(leftJointDynamic, 0)
        end
    else
        --full_search_attempts = 0;
        --search_complete = 0
        straight_back_time = 0
        slope_adjust_time = 0
        search_end_time = 0
        straight_search_slope = 0
        slope_adjust = 0
    end
end

function resetSearchVariables() 
    searched_straight = 0
    searching_straight = 0
    searched_left = 0
    searching_left = 0
    searched_right = 0
    searching_right = 0
    turn_left_undo = 0
    turn_right_undo = 0    
    turningAround = 0
end

function resetSecondarySearchVariables()
    full_search_attempts = 0;
    straight_back_time = 0
    slope_adjust_time = 0
    search_end_time = 0
    straight_search_slope = 0
    slope_adjust = 0
end

-- start path finding search by turning left first
function primarySearch_CheckLeftFirst()
    if(searched_straight == 0 and searching_straight == 0) then
        searching_straight = 1;
        previousTime = sim.getSimulationTime()
    elseif(searching_straight == 1 and (sim.getSimulationTime() - previousTime) < timeToTravelStraight) then
        -- search straight
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius)
        sim.setJointTargetVelocity(leftJointDynamic, nominalLinearVelocity / wheelRadius)
    elseif(searching_straight == 1 and (sim.getSimulationTime() - previousTime) >= timeToTravelStraight) then
        searched_straight = 1;
        searching_straight = 0
        searching_left = 1;
        searched_left = 0;
        -- brake
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        previousTime = sim.getSimulationTime()
    elseif(searching_left == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        -- turn left
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius)
    elseif(searching_left == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        searched_left = 1;
        searching_left = 0;
        turn_left_undo = 1;
        searching_right = 1;
        searched_right = 0;
        previousTime = sim.getSimulationTime()
    elseif(turn_left_undo == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        sim.setJointTargetVelocity(rightJointDynamic, -nominalLinearVelocity / wheelRadius)
    elseif(turn_left_undo == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        turn_left_undo = 0;
        -- brake
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        previousTime = sim.getSimulationTime()
    elseif(searching_right == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        -- turn right
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        sim.setJointTargetVelocity(leftJointDynamic, nominalLinearVelocity / wheelRadius)  
    elseif(searching_right == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        searched_right = 1;
        searching_right = 0;
        turn_right_undo = 1;
        -- brake
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        previousTime = sim.getSimulationTime()
    elseif(turn_right_undo == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        -- undo right turn
        sim.setJointTargetVelocity(leftJointDynamic, -nominalLinearVelocity / wheelRadius) 
    elseif(turn_right_undo == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        turn_right_undo = 0;
        -- brake
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        previousTime = sim.getSimulationTime()
    else
        resetSearchVariables()
        full_search_attempts = full_search_attempts + 1
    end
end

function primarySearch_CheckRightFirst()
    if(searched_straight == 0 and searching_straight == 0) then
        searching_straight = 1;
        previousTime = sim.getSimulationTime()
        -- search straight
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius)
        sim.setJointTargetVelocity(leftJointDynamic, nominalLinearVelocity / wheelRadius)
    elseif(searching_straight == 1 and (sim.getSimulationTime() - previousTime) < timeToTravelStraight) then
        -- search straight
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius)
        sim.setJointTargetVelocity(leftJointDynamic, nominalLinearVelocity / wheelRadius)
    elseif(searching_straight == 1 and (sim.getSimulationTime() - previousTime) >= timeToTravelStraight) then
        searched_straight = 1;
        searching_straight = 0
        searching_right = 1;
        searched_right = 0; -- might be safe to remove since it should be 0
        previousTime = sim.getSimulationTime()
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        sim.setJointTargetVelocity(rightJointDynamic, 0)
    elseif(searching_right == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        -- turn right
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        sim.setJointTargetVelocity(leftJointDynamic, nominalLinearVelocity / wheelRadius)  
    elseif(searching_right == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        searched_right = 1;
        searching_right = 0;
        searching_left = 1;
        searched_left = 0;
        turn_right_undo = 1;
        -- brake
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        previousTime = sim.getSimulationTime()
    elseif(turn_right_undo == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        -- undo right turn
        sim.setJointTargetVelocity(leftJointDynamic, -nominalLinearVelocity / wheelRadius) 
    elseif(turn_right_undo == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        turn_right_undo = 0;
        -- brake
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        previousTime = sim.getSimulationTime()
    elseif(searching_left == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        -- turn left
        sim.setJointTargetVelocity(rightJointDynamic, nominalLinearVelocity / wheelRadius)
    elseif(searching_left == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        searched_left = 1;
        searching_left = 0;
        turn_left_undo = 1;
        previousTime = sim.getSimulationTime()
    elseif(turn_left_undo == 1 and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees) then
        sim.setJointTargetVelocity(rightJointDynamic, -nominalLinearVelocity / wheelRadius)
    elseif(turn_left_undo == 1 and (sim.getSimulationTime() - previousTime) >= timeToTurn90Degrees) then
        turn_left_undo = 0;
        sim.setJointTargetVelocity(leftJointDynamic, 0)
        sim.setJointTargetVelocity(rightJointDynamic, 0)
        previousTime = sim.getSimulationTime()
    else
        resetSearchVariables()
        full_search_attempts = full_search_attempts + 1
    end
end

function sysCall_sensing()
    previous_left_sensor_temp = left_sensor
    previous_right_sensor_temp = right_sensor
    previous_middle_sensor_temp = middle_sensor
    
    left_sensor = sim.readVisionSensor(leftSensor)
    right_sensor = sim.readVisionSensor(rightSensor)
    middle_sensor = sim.readVisionSensor(middleSensor)
    if(previous_left_sensor_temp ~= left_sensor or 
        previous_right_sensor_temp ~= right_sensor or previous_middle_sensor_temp ~= middle_sensor) then
        previous_left_sensor_prev_prev = previous_left_sensor_prev
        previous_right_sensor_prev_prev = previous_right_sensor_prev
        previous_middle_sensor_prev_prev = previous_middle_sensor_prev
        
        previous_left_sensor_prev = previous_left_sensor_temp
        previous_right_sensor_prev = previous_right_sensor_temp
        previous_middle_sensor_prev = previous_middle_sensor_temp
    end
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details