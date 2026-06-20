# Robotika_Menjalankan-Robot-Hexapod
Robot Hexapod Melewati Labirin Tanpa Menabtak Dinding
sim=require'sim'
simIK=require'simIK'

function sysCall_init()
    antBase=sim.getObject('../base')
    legBase=sim.getObject('../legBase')
    
    --------------------------------------------------
    -- NAMA SENSOR 
    --------------------------------------------------
    sensors = {}
    sensors.f1 = sim.getObject('../SENSOR_DEPAN')
    sensors.f2 = sim.getObject('../SENSOR_SERONG_KANAN')
    sensors.r = sim.getObject('../SENSOR_BELAKANG_KANAN')
    sensors.l = sim.getObject('../SENSOR_SERONG_KIRI')
    sensors.fl = sim.getObject('../SENSOR_BELAKANG_KIRI')
    
    --------------------------------------------------
    -- INISIALISASI KAKI
    --------------------------------------------------
    simLegTips={}
    simLegTargets={}
    
    for i=1,6,1 do
        simLegTips[i]=sim.getObject('../footTip'..i-1)
        simLegTargets[i]=sim.getObject('../footTarget'..i-1)
    end

    initialPos={}
    for i=1,6,1 do
        initialPos[i]=sim.getObjectPosition(simLegTips[i], legBase)
    end
    
    legMovementIndex={1,4,2,6,3,5}
    stepProgression=0
    realMovementStrength=0
    
    --------------------------------------------------
    -- IK
    --------------------------------------------------
    ikEnv=simIK.createEnvironment()
    ikGroup=simIK.createGroup(ikEnv)
    local simBase=sim.getObject('..')

    for i=1,#simLegTips,1 do
        simIK.addElementFromScene(
            ikEnv, ikGroup, simBase, simLegTips[i], simLegTargets[i], simIK.constraint_position
        )
    end
    
    --------------------------------------------------
    -- PARAMETER ROBOT
    --------------------------------------------------
    sizeFactor=sim.getObjectSizeFactor(antBase)
    walkingVel=1.5
    maxWalkingStepSize=0.1*sizeFactor 
    stepHeight=0.04*sizeFactor
    
    --------------------------------------------------
    -- PARAMETER PID 
    --------------------------------------------------
    wallDistTarget = 0.28
    safeDistForward = 0.6
    Kp = 2.5
    Ki = 0.01
    Kd = 0.5
    prevError = 0
    integral = 0
    
    --------------------------------------------------
    -- MOVEMENT & DETEKSI DUMMY FINISH
    --------------------------------------------------
    movData={}
    setStepMode(walkingVel, maxWalkingStepSize, stepHeight, 0, 0, 0)

    isFinished = false
    
    -- Mencari objek Dummy di base utama scene luar
    targetFinish = sim.getObject('/FinishPoint')
    if not targetFinish then
        print("PERINGATAN: Objek Dummy bernama 'FinishPoint' belum dibuat di scene!")
    end
    
    -- Jarak toleransi (meter) antara robot dengan Dummy untuk memicu berhenti
    stopRadius = 0.35 
end

--------------------------------------------------
-- ACTUATION
--------------------------------------------------
function sysCall_actuation()
    dt=sim.getSimulationTimeStep()
    
    --------------------------------------------------
    -- CEK STATUS SELESAI
    --------------------------------------------------
    if isFinished then
        setStepMode(0, 0, 0, 0, 0, 0)
        -- Tetap jalankan IK agar robot berdiri kokoh, tidak lemas
        simIK.handleGroup(ikEnv, ikGroup, {syncWorlds=true, allowError=true})
        return
    end
    
    --------------------------------------------------
    -- LOGIKA DETEKSI JARAK KE DUMMY FINISH
    --------------------------------------------------
    if targetFinish then
        local robotPos = sim.getObjectPosition(antBase, -1)
        local targetPos = sim.getObjectPosition(targetFinish, -1)
        
        -- Hitung jarak geometri antara robot dan posisi Dummy
        local distance = math.sqrt((robotPos[1] - targetPos[1])^2 + (robotPos[2] - targetPos[2])^2)
        
        if distance < stopRadius then
            isFinished = true
            print("Robot berhasil mencapai objek FinishPoint! Robot berhenti total.")
        end
    end
    
    --------------------------------------------------
    -- BACA SENSOR
    --------------------------------------------------
    local rF1,dF1=sim.readProximitySensor(sensors.f1)
    if rF1 == 0 then dF1 = 2.0 end
    
    local rF2,dF2=sim.readProximitySensor(sensors.f2)
    if rF2 == 0 then dF2 = 2.0 end
    
    local rR,dR=sim.readProximitySensor(sensors.r)
    if rR == 0 then dR = 2.0 end
    
    local rL,dL=sim.readProximitySensor(sensors.l)
    if rL == 0 then dL = 2.0 end
    
    local rFL,dFL=sim.readProximitySensor(sensors.fl)
    if rFL == 0 then dFL = 2.0 end
    
    local tDir = 0
    local tRot = 0
    local tStr = 1

    --------------------------------------------------
    -- KINEMATIC FORCE
    --------------------------------------------------
    local minFrontDist = math.min(dF1,dF2,dFL)
    local repulsiveForce = 0
    
    if minFrontDist < safeDistForward then
        --------------------------------------------------
        -- GAYA TOLAK & BELOK KANAN
        --------------------------------------------------
        repulsiveForce = (safeDistForward - minFrontDist) / safeDistForward
        tRot = -1.5 * repulsiveForce 
        tDir = -10 * repulsiveForce
        tStr = math.max(0.2, 1.0 - repulsiveForce)
        integral = 0 
    
    --------------------------------------------------
    -- PID WALL FOLLOW
    --------------------------------------------------
    elseif rL == 1 then
        local error = wallDistTarget - dL
        integral = integral + (error * dt)
        local derivative = (error - prevError) / dt
        local pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative)
        prevError = error
        
        tRot = -pidOutput
        tDir = -pidOutput * 25
        tStr = 1.0
        
    else
        --------------------------------------------------
        -- SEARCH MODE
        --------------------------------------------------
        tDir = 15
        tRot = 0.2
        tStr = 0.8
        prevError = 0
        integral = 0
    end
    
    --------------------------------------------------
    -- STEP MODE
    --------------------------------------------------
    setStepMode(walkingVel, maxWalkingStepSize, stepHeight, tDir, tRot, tStr)

    --------------------------------------------------
    -- IK MOVEMENT
    --------------------------------------------------
    dx = movData.strength - realMovementStrength

    if (math.abs(dx) > dt*0.1) then
        dx = math.abs(dx)*dt*0.5/dx
    end

    realMovementStrength = realMovementStrength + dx
    
    for leg=1,6,1 do
        local sp = (stepProgression + (legMovementIndex[leg]-1)/6) % 1
        local offset = {0,0,0}

        if (sp < (1/3)) then
            offset[1] = sp*3*movData.amplitude/2
        elseif (sp < (1/2)) then 
            local s = sp-1/3
            offset[1] = movData.amplitude/2 - movData.amplitude*s*6/2
            offset[3] = s*6*movData.height
        elseif (sp < (2/3)) then
            local s = sp-1/2
            offset[1] = -movData.amplitude*s*6/2
            offset[3] = (1-s*6)*movData.height
        else
            local s = sp-2/3
            offset[1] = -movData.amplitude*(1-s*3)/2
        end

        local md = movData.dir + math.abs(movData.rot) * math.atan2(initialPos[leg][1]*movData.rot, -initialPos[leg][2]*movData.rot)

        local offset2 = {
            offset[1] * math.cos(md) * realMovementStrength,
            offset[1] * math.sin(md) * realMovementStrength,
            offset[3] * realMovementStrength
        }

        local p = {
            initialPos[leg][1] + offset2[1],
            initialPos[leg][2] + offset2[2],
            initialPos[leg][3] + offset2[3]
        }

        sim.setObjectPosition(simLegTargets[leg], p, legBase)
    end

    --------------------------------------------------
    -- HANDLE IK
    --------------------------------------------------
    simIK.handleGroup(ikEnv, ikGroup, {syncWorlds=true, allowError=true})
    stepProgression = stepProgression + dt*movData.vel
end

--------------------------------------------------
-- STEP MODE
--------------------------------------------------
function setStepMode(vel, amp, h, dir, rot, str)
    movData.vel=vel
    movData.amplitude=amp
    movData.height=h
    movData.dir=math.pi*dir/180
    movData.rot=rot
    movData.strength=str
end

--------------------------------------------------
-- THREAD
--------------------------------------------------
function sysCall_thread()
    local initialP={0, 0, -0.03 * sim.getObjectSizeFactor(antBase)}
    local initialO={0, 0, 0}

    local params = {
        object=legBase, 
        relObject=antBase, 
        targetPose=sim.buildPose(initialP, initialO), 
        maxVel={0.1, 0.1, 0.1, 0.1}, 
        maxAccel={0.1, 0.1, 0.1, 0.1},
        maxJerk={0.1, 0.1, 0.1, 0.1}
    }

    sim.moveToPose(params)

    while true do
        sim.switchThread()
    end
end
