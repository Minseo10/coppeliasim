local function findIndex(arr, value)
    for i, v in ipairs(arr) do
        if v == value then
            return i
        end
    end
    return nil
end

function rad2degree(radianValue)
    return radianValue * 180 / math.pi
end

function sysCall_init()
    sim = require('sim')
    simIK=require'simIK'
    
    -- do some initialization here
    jointHandles = {}
    for i = 1, 6, 1 do
        jointHandles[i] = sim.getObject('../joint', {index = i - 1})
    end
    
    currentJointIndex = 1
end

function sysCall_actuation()
    -- put your actuation code here
    msg, auxData = sim.getSimulatorMessage()
    dx = 0.1
    
    local qwerty = {113, 119, 101, 114, 116, 121}
    
    if msg == 6 then -- key press
        key = auxData[1]
        
        newJointIndex = findIndex(qwerty, key)
        if newJointIndex then
            currentJointIndex = newJointIndex
            print('[Right] current joint index:', currentJointIndex)
        end
        
        local currentPosition = sim.getJointPosition(jointHandles[currentJointIndex])
        
        if key == 91 then
            sim.setJointTargetPosition(jointHandles[currentJointIndex], currentPosition-dx)
            print('[Right] [ pressed, ', rad2degree(currentPosition))
        end
        
        if key == 93 then
            sim.setJointTargetPosition(jointHandles[currentJointIndex], currentPosition+dx)
            print('[Right] ] pressed, ', rad2degree(currentPosition))
        end
        
        if key == 2001 then -- enter
            for i = 1, 6, 1 do
                joint = sim.getJointPosition(jointHandles[i])
                print('[Right] ', i, rad2degree(joint))
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
