function rad2degree(radianValue)
    return radianValue * 180 / math.pi
end

function degree2rad(degreeValue)
    return degreeValue * math.pi / 180
end

setGripperData=function(open,velocity,force)
    if not velocity then
        velocity=0.11
    end
    if not force then
        force=20
    end
    if not open then
        velocity=-velocity
    end
    
    local dat={}
    dat.velocity=velocity
    dat.force=force
    sim.writeCustomBufferData(gripperHandle,'activity',sim.packTable(dat))
end

function moveToPose_viaIK(maxVelocity,maxAcceleration,maxJerk,targetQ,auxData)
    local params = {
        ik = {tip = auxData.tip, target = auxData.target, base = auxData.base},
        targetPose = targetQ,
        maxVel = maxVelocity,
        maxAccel = maxAcceleration,
        maxJerk = maxJerk,
    }
    sim.moveToPose(params)
end

function moveToConfig_viaFK(maxVelocity,maxAcceleration,maxJerk,goalConfig,auxData)
    local params = {
        joints = auxData.joints,
        targetPos = goalConfig,
        maxVel = maxVelocity,
        maxAccel = maxAcceleration,
        maxJerk = maxJerk,
    }
    sim.moveToConfig(params)
end

function sysCall_init()
    sim=require'sim'
    simIK=require'simIK'
    
    -- do some initialization here
    
    simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObject('../joint',{index=i-1})
    end
    
    simTip=sim.getObject('../ikTip')
    simTarget=sim.getObject('../ikTarget')
    modelBase=sim.getObject('..')
    gripperHandle=sim.getObject('../RG2')
    
    -- FK movement data:
    initConf={0,0,0,0,0,0}
    vel=180
    accel=40
    jerk=80
    maxVel={
        vel*math.pi/180,
        vel*math.pi/180,
        vel*math.pi/180,
        vel*math.pi/180,
        vel*math.pi/180,
        vel*math.pi/180
    }
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}

    -- IK movement data:
    ikMaxVel={0.4,0.4,0.4,1.8}
    ikMaxAccel={0.8,0.8,0.8,0.9}
    ikMaxJerk={0.6,0.6,0.6,0.8}

    pickConfig={-70.1*math.pi/180,18.85*math.pi/180,93.18*math.pi/180,68.02*math.pi/180,109.9*math.pi/180,90*math.pi/180}

    dropConfig1={-183.34*math.pi/180,14.76*math.pi/180,78.26*math.pi/180,-2.98*math.pi/180,-90.02*math.pi/180,86.63*math.pi/180}
    dropConfig2={-197.6*math.pi/180,14.76*math.pi/180,78.26*math.pi/180,-2.98*math.pi/180,-90.02*math.pi/180,72.38*math.pi/180}
    dropConfig3={-192.1*math.pi/180,3.76*math.pi/180,91.16*math.pi/180,-4.9*math.pi/180,-90.02*math.pi/180,-12.13*math.pi/180}
    dropConfig4={-189.38*math.pi/180,24.94*math.pi/180,64.36*math.pi/180,0.75*math.pi/180,-90.02*math.pi/180,-9.41*math.pi/180}

    dropConfigs={dropConfig1,dropConfig2,dropConfig3,dropConfig4}
    dropConfigIndex=1
    droppedPartsCnt=0

    setGripperData(true)
    sim.setInt32Param(sim.intparam_current_page, 0)

    data={}
    data.tip=simTip
    data.target=simTarget
    data.base = modelBase
    data.joints=simJoints
    
    stage = 1
    lastCallTime = sim.getSimulationTime()
    
    ------ start of my own code
    currentJointIndex = 1
    ------ end of my own code
end

function sysCall_actuation()
    local currentTime = sim.getSimulationTime()
    local elapsedTime = currentTime - lastCallTime

    if stage == 1 and elapsedTime > 0.0 then
        moveToConfig_viaFK(maxVel,maxAccel,maxJerk,pickConfig,data)

        local pose=sim.getObjectPose(simTip)
        pose[1]=pose[1]+0.105
        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)

        setGripperData(false)
        
        stage = stage + 1
        
    end
    if stage == 2 and elapsedTime > 2.0 then

        moveToConfig_viaFK(maxVel,maxAccel,maxJerk,initConf,data)
        stage = stage + 1
    end

    if stage == 3 and elapsedTime > 4.0 then
        setGripperData(false)
        print('setGripperData(false)')
        stage = stage + 1
    end
    if stage == 4 and elapsedTime > 6.0 then
        setGripperData(true)
        print('setGripperData(true)')
        stage = stage + 1
    end
    if stage == 5 and elapsedTime > 8.0 then
        setGripperData(false)
        print('setGripperData(false)')
        stage = stage + 1
    end
    if stage == 6 and elapsedTime > 10.0 then
        setGripperData(true)
        print('setGripperData(true)')
        stage = stage + 1
    end

    ------ start of my own code
    msg, auxData = sim.getSimulatorMessage()
    dx = 0.1
    
    if msg == 6 then -- key press
        key = auxData[1]
        --print(key)
        
        if key >= 49 and key <= 54 then -- 1~6
            currentJointIndex = key - 48
            print('[Left] current joint index:', currentJointIndex)
        end
        
        local currentPosition = sim.getJointPosition(simJoints[currentJointIndex])
        
        if key == 45 then
            sim.setJointTargetPosition(simJoints[currentJointIndex], currentPosition-dx)
            print('[Left] - pressed, ', rad2degree(currentPosition))
        end
        
        if key == 61 then
            sim.setJointTargetPosition(simJoints[currentJointIndex], currentPosition+dx)
            print('[Left] + pressed, ', rad2degree(currentPosition))
        end
        
        if key == 2001 then -- enter
            for i = 1, 6, 1 do
                joint = sim.getJointPosition(simJoints[i])
                print('[Left] ', i, rad2degree(joint))
            end
        end
    end
    ------ end of my own code

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
