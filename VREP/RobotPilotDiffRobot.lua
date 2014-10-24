function RobotPilotSimulationLoop()
	while (simGetSimulationState() ~= sim_simulation_advancing_abouttostop) do

		-- Get Robot, Rabit Configuration
		position = simGetObjectPosition(objHandle,sim_handle_parent)
		orientation = simGetObjectOrientation(objHandle,sim_handle_parent)		
		position2 = simGetObjectPosition(dummy,sim_handle_parent)
		orientation2 = simGetObjectOrientation(dummy,sim_handle_parent)
		dataOut={0.0, 0.0, (position[1] * 1000.0), (position[2] * 1000.0), orientation2[3]}		
		
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

		-- Set Dummmy Position
		position2[1] = returnData[1] / 1000
		position2[2] = returnData[2] / 1000

		orientation2[1] = 0
		orientation2[2] = 0
		orientation2[3] = returnData[3]
									 
		simSetObjectPosition(objHandle,sim_handle_parent,position)
		simSetObjectOrientation(objHandle,sim_handle_parent,orientation)							
		simSetObjectPosition(dummy, sim_handle_parent, position2)
		simSetObjectOrientation(dummy, sim_handle_parent , orientation2)			
		
		simHandleGraph(graphHandle, simGetSimulationTime()+simGetSimulationTimeStep())
  
		-- Now don't waste time in this loop if the simulation time hasn't changed! This also synchronizes this thread with the main script
		simSwitchThread() -- This thread will resume just before the main script is called again
	end
end

function RobotPilotDiffRobot()
	objHandle=simGetObjectHandle("diff_robot_ext") -- Handle of the robot
	start = loadOBJ("c:/BME/Diploma/Src/RobotSolution/Frame/frame" .. envFile ..  ".obj")
	dummy = simCreateDummy(0.01)
	simSetObjectPosition(dummy,sim_handle_parent,{start[1], start[2], 0.002})
	simSetObjectOrientation(dummy,sim_handle_parent,{0, 0, start[3]})		
	robotShape = simGetObjectHandle("Robot") -- Handle of the robot shape
	simSetObjectParent(robotShape, dummy, true)	
	simSetObjectPosition(objHandle,sim_handle_parent,{start[1], start[2], 0.09})
	simSetObjectOrientation(objHandle,sim_handle_parent,{0, 0, start[3]})
	
	--Turn off dynamic for Robot
	p=simBoolOr16(simGetModelProperty(objHandle),sim_modelproperty_not_dynamic)
	simSetModelProperty(objHandle,p)		

	--Turn off dynamic for Robot Shape
	p = simBoolOr16(simGetModelProperty(robotShape),sim_modelproperty_not_dynamic)
	simSetModelProperty(robotShape, p)			

	-- We start the server on a port that is probably not used:
	local portNb = getPort()

	graphHandle = simGetObjectHandle("Graph")
	simResetGraph(graphHandle)	
	
	-- Start Server
	result=simLaunchExecutable('ServerVrepApp',portNb,1) -- set the last argument to 1 to see the console of the launched server

	if ((result == -1)) then
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
				createPath(simUnpackFloats(returnData))

				-- We could connect to the server
				RobotPilotSimulationLoop()
				--simStopSimulation()													
			end
			client:close()
		end
	end
end