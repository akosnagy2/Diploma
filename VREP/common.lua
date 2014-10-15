-- Collects the paths int the scene
function collectPath()
	local paths = {}
	local tmpHnd
	local i = 0
	repeat
		tmpHnd = simGetObjectHandle("Path" .. i) -- Handle of the geometric path
		if(tmpHnd ~= -1) then
			paths[tmpHnd] = false
			i = i + 1
		else
			tmpHnd = simGetObjectHandle("Path" .. i .."B")
			if(tmpHnd ~= -1) then
				paths[tmpHnd] = true
				i = i + 1
			end
		end
	until tmpHnd == -1
	return paths
end

-- Sample path
function samplePath(paths, step)
	local pathPoints = {}
	local j = 1
	local i = 0
	for pHandle, pDir in pairs(paths) do
		for i = 0, 1 + step, step do
			local pos = simGetPositionOnPath(pHandle,i)
			local ori = simGetOrientationOnPath(pHandle,i)
			pathPoints[j] = pos[1] * 1000.0 --x
			j = j + 1
			pathPoints[j] = pos[2] * 1000.0 --y
			j = j + 1
			if(pDir) then
				pathPoints[j] = ori[2] - math.pi/2 + math.pi;
			else
				pathPoints[j] = ori[2] - math.pi/2;
			end
			j = j + 1
		end
	end
	return pathPoints
end

-- GetFreePort
function getFreePort()
	simSetThreadAutomaticSwitch(false)
	local portNb=simGetIntegerParameter(sim_intparam_server_port_next)
	local portStart=simGetIntegerParameter(sim_intparam_server_port_start)
	local portRange=simGetIntegerParameter(sim_intparam_server_port_range)
	local newPortNb=portNb+1
	if (newPortNb>=portStart+portRange) then
		newPortNb=portStart
	end

	simSetIntegerParameter(sim_intparam_server_port_next,newPortNb)
	simSetThreadAutomaticSwitch(true)
	return portNb
end

function createPath(data)
	local sampledPath = simCreatePath(-1,NULL,NULL,NULL)
	local ptData = {}		
	print("Sampled path received.Length: " .. table.getn(data)/2)
	for i = 0, table.getn(data)/2 - 1  do
		ptData[1] = data[i*2+1] / 1000.0 --[mm] -> [m]
		ptData[2] = data[i*2+2] / 1000.0 --[mm] -> [m]
		ptData[3] = 0.0
		ptData[4] = 0.0
		ptData[5] = 0.0
		ptData[6] = 0.0
		ptData[7] = 1.0
		ptData[8] = 0.0
		ptData[9] = 1
		ptData[10] = 0.5
		ptData[11] = 0.5
		simInsertPathCtrlPoints(sampledPath,0,i,1,ptData)
	end
end
