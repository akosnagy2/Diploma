function PathPlannerSimulationLoop()
	while (simGetSimulationState() ~= sim_simulation_advancing_abouttostop) do
	
		phiLeft = simGetJointPosition(leftMotor)
		phiRight = simGetJointPosition(rightMotor)

		-- Get Robot, Rabit Configuration
		rabitPosition = simGetObjectPosition(rabitHandle,sim_handle_parent)	
		position = simGetObjectPosition(objHandle,sim_handle_parent)
		orientation = simGetObjectOrientation(objHandle,sim_handle_parent)		
		position2 = simGetObjectPosition(dummy,sim_handle_parent)
		orientation2 = simGetObjectOrientation(dummy,sim_handle_parent)
		dataOut={phiLeft, phiRight, (position[1] * 1000.0), (position[2] * 1000.0), orientation2[3]}		
		
		-- Send the data
		dataOut=simPackFloats(dataOut)
		writeSocketData(client,dataOut)
		
		-- Read the reply from the server:
		local returnData=readSocketData(client)
		if (returnData==nil) then
			break -- Read error
		end
		-- unpack the received data:
		returnData=simUnpackFloats(returnData)
		
		-- Set Robot Configuration
		position[1] = returnData[1] / 1000
		position[2] = returnData[2] / 1000
		orientation[3] = returnData[3]
		
		-- Set Robot Motors Angle
		lj = returnData[4]
		rj = returnData[5]

		-- Set Rabit Position
		rabitPosition[1] = returnData[6] /1000
		rabitPosition[2] = returnData[7] /1000

		-- Set Dummmy Position
		position2[1] = returnData[1] / 1000
		position2[2] = returnData[2] / 1000

		orientation2[1] = 0
		orientation2[2] = 0
		orientation2[3] = returnData[3]
					
		-- Set graph, robot, rabit position
		simSetGraphUserData(trackGraphHandle, "TrackError", returnData[8]) 
		simSetGraphUserData(trackGraphHandle, "PredictError", returnData[9]) 
		simAuxiliaryConsolePrint(consoleHandle,NULL) 
		simAuxiliaryConsolePrint(consoleHandle,"Tracking Error: " .. returnData[8] .. "mm\r\n") 
		simAuxiliaryConsolePrint(consoleHandle,"Tracking Sum Error: " .. returnData[9] .. "mm\r\n") 
		
		if (dh ~= nil) then
			simRemoveDrawingObject(dh)	
		end
		dh = drawLine({returnData[10]/1000, returnData[11]/1000, 0.1}, 					
					  {returnData[12]/1000, returnData[13]/1000, 0.1})
				 
		simSetObjectPosition(objHandle,sim_handle_parent,position)
		simSetObjectOrientation(objHandle,sim_handle_parent,orientation)					
		simSetObjectPosition(rabitHandle,sim_handle_parent,rabitPosition)			
		simSetObjectPosition(dummy, sim_handle_parent, position2)
		simSetObjectOrientation(dummy, sim_handle_parent , orientation2)
		
		simSetJointPosition(leftMotor,lj)	
		simSetJointPosition(rightMotor,rj)		

		simHandleGraph(graphHandle, simGetSimulationTime()+simGetSimulationTimeStep())
		
		-- Now don't waste time in this loop if the simulation time hasn't changed! This also synchronizes this thread with the main script
		simSwitchThread() -- This thread will resume just before the main script is called again
	end
end

function PathPlannerDiffRobot()
	objHandle=simGetObjectHandle("diff_robot_ext") -- Handle of the robot
	start = loadOBJ("c:/BME/Diploma/Src/RobotSolution/Frame/frame" .. envFile ..  ".obj")
	dummy = simCreateDummy(0.01)
	simSetObjectPosition(dummy,sim_handle_parent,{start[1], start[2], 0.002})
	simSetObjectOrientation(dummy,sim_handle_parent,{0, 0, start[3]})		
	robotShape = simGetObjectHandle("Robot") -- Handle of the robot shape
	simSetObjectParent(robotShape, dummy, true)	
	simSetObjectPosition(objHandle,sim_handle_parent,{start[1], start[2], 0.09})
	simSetObjectOrientation(objHandle,sim_handle_parent,{0, 0, start[3]})

	-- Get some handles first
	leftMotor=simGetObjectHandle("leftWheelJoint") -- Handle of the left motor
	rightMotor=simGetObjectHandle("rightWheelJoint") -- Handle of the right motor
	trackGraphHandle=simGetObjectHandle("TrackErrorGraph")
	consoleHandle=simAuxiliaryConsoleOpen("Console",10,0,NULL,NULL,NULL)
	rabitHandle=simGetObjectHandle("rabit") -- Handle of the robot
	
	--Turn off dynamic for Robot
	p=simBoolOr16(simGetModelProperty(objHandle),sim_modelproperty_not_dynamic)
	simSetModelProperty(objHandle,p)	
	
	--Turn off dynamic for Rabit
	p=simBoolOr16(simGetModelProperty(rabitHandle),sim_modelproperty_not_dynamic)
	simSetModelProperty(rabitHandle,p)

	--Turn off dynamic for Robot Shape
	p = simBoolOr16(simGetModelProperty(robotShape),sim_modelproperty_not_dynamic)
	simSetModelProperty(robotShape, p)			

	-- We start the server on a port that is probably not used:
	local portNb = getPort()

	graphHandle = simGetObjectHandle("Graph")
	simResetGraph(graphHandle)		
	
	-- Start Server
	result=simLaunchExecutable('ServerVrepApp',portNb,1) -- set the last argument to 1 to see the console of the launched server

	-- Start Client
	result2=simLaunchExecutable('PathPlannerApp',"",1) -- set the last argument to 1 to see the console of the launched server

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
								robotMultFactorL, 
								robotSmoothFactor, 	
								robotMaxSpeed, 
								robotMaxAccel, 
								robotMultFactorR, 
								robotSmoothFactor, 
								predictSampleLength, 
								predictDistanceLength,					
								oriPar_P, 
								oriPar_D, 
								wheelDistance, 
								wheelDiameter,
								pathMaxSpeed,
								pathMaxAccel,
								pathMaxTangentAccel,
								pathMaxAngularSpeed,
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
				
				-- Read sampled path				
				local returnData=readSocketData(client)
				if (returnData==nil) then
					break -- Read error
				end

				-- Create sampled path
				local pathColor1 = {
					0.0,0.75,0.0,
					0.25,0.25,0.25,
					0.25,0.25,0.25,
					0.0,0.0,0.0
				}				
				createPath(simUnpackFloats(returnData), pathColor1)
				
				-- Read sampled path
				local returnData=readSocketData(client)
				if (returnData==nil) then
					break -- Read error
				end

				-- Create sampled path
				local pathColor2 = {
					1.0,0.0,0.0,
					0.45,0.45,0.45,
					0.45,0.45,0.45,
					0.0,0.0,0.0
				}	
				createPath(simUnpackFloats(returnData), pathColor2)	

				-- We could connect to the server
				PathPlannerSimulationLoop()
				simStopSimulation()													
			end
			client:close()
		end
	end
end