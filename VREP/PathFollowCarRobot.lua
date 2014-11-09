function PathFollowSimulationLoop()
	while (simGetSimulationState() ~= sim_simulation_advancing_abouttostop) do

		phiLeft=simGetJointPosition(leftMotor)
		phiRight=simGetJointPosition(rightMotor)

		position=simGetObjectPosition(objHandle,sim_handle_parent)
		orientation=simGetObjectOrientation(objHandle,sim_handle_parent) 

		-- Prepare the data to send:
		local dataOut={phiLeft, phiRight, (position[1] * 1000.0), (position[2] * 1000.0), orientation[3]} --[mm] -> [m]
		-- Pack the data as a string:
		dataOut=simPackFloats(dataOut)
		-- Send the data:
		writeSocketData(client,dataOut)
		-- Read the reply from the server:
		local returnData=readSocketData(client)
		if (returnData==nil) then
			break -- Read error
		end
		-- unpack the received data:
		returnData=simUnpackFloats(returnData)
		pos = {}
		ori = {}
		rabitPos = {}

		pos[1] = returnData[1] / 1000.0 --[mm] -> [m]
		pos[2] = returnData[2] / 1000.0 --[mm] -> [m]
		pos[3] = position[3] -- fix

		ori[1] = orientation[1] -- fix
		ori[2] = orientation[2] -- fix
		ori[3] = returnData[3]

		v = returnData[4] / 1000.0
		fi = returnData[5]

		rabitPos[1] = returnData[6] / 1000
		rabitPos[2] = returnData[7] / 1000
		rabitPos[3] = 0.1 -- fix
		
		simSetGraphUserData(trackGraphHandle, "Steering_Angle", returnData[8]) 

		if(consoleHandle ~= nil) then
			simAuxiliaryConsolePrint(consoleHandle,NULL) 
			simAuxiliaryConsolePrint(consoleHandle,"Tracking Error: " .. returnData[8] .. "mm\r\n") 
			simAuxiliaryConsolePrint(consoleHandle,"Tracking Sum Error: " .. returnData[9] .. "mm\r\n")
		end

		simSetObjectPosition(objHandle,sim_handle_parent,pos)
		simSetObjectOrientation(objHandle,sim_handle_parent,ori)

		setRobotSpeed(v,fi)

		simSetObjectPosition(rabitHandle,sim_handle_parent,rabitPos)
		
		simHandleGraph(graphHandle, simGetSimulationTime()+simGetSimulationTimeStep())
		simHandleGraph(trackGraphHandle, simGetSimulationTime()+simGetSimulationTimeStep())
		
		-- Now don't waste time in this loop if the simulation time hasn't changed! This also synchronizes this thread with the main script
		simSwitchThread() -- This thread will resume just before the main script is called again
	end
end

function PathFollowCarRobot()
	-- Handles
	--consoleHandle=simAuxiliaryConsoleOpen("Console",10,0,NULL,NULL,NULL)

	objHandle = simGetObjectAssociatedWithScript(sim_handle_self)
	leftMotor=simGetObjectHandle("BackLeftMotor") -- Handle of the left motor
	rightMotor=simGetObjectHandle("BackRightMotor") -- Handle of the right motor
	leftSteer=simGetObjectHandle("FrontLeftTurnerJoint") -- Handle of the front left turner joint
	rightSteer=simGetObjectHandle("FrontRightTurnerJoint") -- Handle of the front right turner joint
	rabitHandle=simGetObjectHandle("Sphere") -- Handle of the rabit (path position + prediction length)
	trackGraphHandle=simGetObjectHandle("TrackErrorGraph")
	
	graphHandle = simGetObjectHandle("Graph")
	simResetGraph(graphHandle)
	simResetGraph(trackGraphHandle)
	
	--Turn off dynamic for Robot
	p=simBoolOr16(simGetModelProperty(objHandle),sim_modelproperty_not_dynamic)
	simSetModelProperty(objHandle,p)	
	
	--Turn off dynamic for Rabit
	s=simBoolOr16(simGetModelProperty(rabitHandle),sim_modelproperty_not_dynamic)
	simSetModelProperty(rabitHandle,s)

	-- Get path from Scene
	local path = samplePath(collectPath(), step)

	-- We start the server on a port that is probably not used:
	local portNb = getPort()

	-- Start Server
	result=simLaunchExecutable('ServerVrepApp',portNb,0) -- set the last argument to 1 to see the console of the launched server

	-- Start Client
	result2=simLaunchExecutable('PathPlannerApp',"",0) -- set the last argument to 1 to see the console of the launched server

	if ((result == -1) or (result2 == -1)) then
		-- The executable could not be launched!
		simDisplayDialog('Error',"'ServerVrepApp' could not be launched. &&nSimulation will not run properly",sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
	else
		while (simGetSimulationState()~=sim_simulation_advancing_abouttostop) do
			-- The executable could be launched. Now build a socket and connect to the server:
			local socket=require("socket")
			client=socket.tcp()
			simSetThreadIsFree(true) -- To avoid a bief moment where the simulator would appear frozen
			local result=client:connect('127.0.0.1',portNb)
			simSetThreadIsFree(false)

			if (result==1) then
				-- Send parameters
				local pars  = {	program,
								dt,
								robotMaxSpeed,
								robotMaxAccel, 
								robotMultFactor, 
								robotSmoothFactor,
								robotSteerSpeed,
								predictLength,					
								distPar_P, 
								distPar_D, 
								w0, 
								ksi, 
								wheelDistance, 
								wheelDiameter,
								pathMaxSpeed,
								pathMaxAccel,
								axisDistance,
								fiMax,
								iterationMax,
								fixPathProb,
								roadmapProb,
								randomSeed,
								envFile,
								minimumRadius,
								ds		
								}
				pars=simPackFloats(pars)
				writeSocketData(client,pars)

				-- Send geometric path
				pathOut=simPackFloats(path)
				writeSocketData(client,pathOut)					
				
				-- Read sampled path
				local returnData=readSocketData(client)
				if (returnData==nil) then
					break -- Read error
				end

				-- Create sampled path
				createPath(simUnpackFloats(returnData))

				-- We could connect to the server
				PathFollowSimulationLoop()
				simStopSimulation()					
			end
			client:close()
		end
	end
end