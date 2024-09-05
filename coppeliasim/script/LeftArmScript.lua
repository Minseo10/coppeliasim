function rad2degree(radianValue)
    return radianValue * 180 / math.pi
end

function degree2rad(degreeValue)
    return degreeValue * math.pi / 180
end


function sysCall_init()
    sim = require('sim')
    -- do some initialization here
    
    jointHandles = {}
    for i = 1, 6, 1 do
        jointHandles[i] = sim.getObject('../joint', {index = i - 1})
    end
    
    currentJointIndex = 1
    
    simTip=sim.getObject('../ikTip')
    simTarget=sim.getObject('../ikTarget')
    modelBase=sim.getObject('..')
    gripperHandle=sim.getObject('../RG2')
    
    data={}
    data.tip=simTip
    data.target=simTarget
    data.base = modelBase
    data.joints=simJoints
end

function sysCall_actuation()
    -- put your actuation code here
    msg, auxData = sim.getSimulatorMessage()
    dx = 0.1
    
    if msg == 6 then -- key press
        key = auxData[1]
        --print(key)
        
        if key >= 49 and key <= 54 then -- 1~6
            currentJointIndex = key - 48
            print('[Left] current joint index:', currentJointIndex)
        end
        
        local currentPosition = sim.getJointPosition(jointHandles[currentJointIndex])
        
        if key == 45 then
            sim.setJointTargetPosition(jointHandles[currentJointIndex], currentPosition-dx)
            print('[Left] - pressed, ', rad2degree(currentPosition))
        end
        
        if key == 61 then
            sim.setJointTargetPosition(jointHandles[currentJointIndex], currentPosition+dx)
            print('[Left] + pressed, ', rad2degree(currentPosition))
        end
        
        if key == 2001 then -- enter
            for i = 1, 6, 1 do
                joint = sim.getJointPosition(jointHandles[i])
                print('[Left] ', i, rad2degree(joint))
            end
        end
    end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
