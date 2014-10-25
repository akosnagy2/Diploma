-- Collects the paths int the scene
function collectPath()
	local paths = {}
	local tmpHnd
	local i = 0
	repeat
		tmpHnd = simGetObjectHandle("Path" .. i) -- Handle of the geometric path
		if(tmpHnd ~= -1) then
			paths[tmpHnd] = 1.0
			i = i + 1
		else
			tmpHnd = simGetObjectHandle("Path" .. i .."B")
			if(tmpHnd ~= -1) then
				paths[tmpHnd] = 0.0
				i = i + 1
			end
		end
	until tmpHnd == -1
	return paths
end

-- Sample path, dir=true -> forward, dir=false -> backward
function samplePath(paths, step)
	local pathOut = {}
	local j = 1
	local i = 0
	for pHandle, pDir in pairs(paths) do
		table.insert(pathOut, pDir) --path dir
		print(pDir)
		table.insert(pathOut, math.ceil(1/step) + 1) --path size	
		for i = 0, 1 + step, step do
			local pos = simGetPositionOnPath(pHandle,i)
			local ori = simGetOrientationOnPath(pHandle,i)
			table.insert(pathOut, pos[1] * 1000)
			table.insert(pathOut, pos[2] * 1000)
			table.insert(pathOut, ori[2] - math.pi/2)
		end
	end
	return pathOut
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

function createPath(data, pathColor)
	local sampledPath = simCreatePath(-1,{2,sim_distcalcmethod_dl,0},nil,pathColor)
	if sampledPath == -1 then return end
	local ptData = {}		
	print("Sampled path received.Length: " .. table.getn(data)/2)
	for i = 0, table.getn(data)/2 - 1  do
		ptData[1] = data[i*2+1] / 1000.0 --[mm] -> [m]
		ptData[2] = data[i*2+2] / 1000.0 --[mm] -> [m]
		ptData[3] = 0.1
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

-- Following function writes data to the socket (only single packet data for simplicity sake):
writeSocketData=function(client,data)
	local header=string.char(50,47,math.mod(#data,256),math.floor(#data/256),0,0)
	-- Packet header is (in this case): headerID (50,47), dataSize (WORD), packetsLeft (WORD) but not used here
	client:send(header..data)
end

-- Following function reads data from the socket (only single packet data for simplicity sake):
readSocketData=function(client)
	-- Packet header is: headerID (50,47), dataSize (WORD), packetsLeft (WORD) but not used here
	local header=client:receive(6)
	if (header==nil) then
		return(nil) -- error
	end
	if (header:byte(1)==50)and(header:byte(2)==47) then
		local l=header:byte(3)+header:byte(4)*256
		return(client:receive(l))
	else
		return(nil) -- error
	end
end

function loadOBJ(filename)
    vertices,indices,reserved,names=simImportMesh(0,filename,3,0,0.001)
	start = {}
    if (vertices) then
        for i=1,#vertices,1 do
			if (names[i] == "StartConfig") then				
				start[1] = vertices[i][1]	--x
				start[2] = vertices[i][2]	--y
				start[3] = vertices[i][3] * 1000 --phi
			elseif (names[i] == "Robot") then
				--Transformation matrix
				matrix = {}
				matrix[1] = math.cos(start[3])
				matrix[2] = -math.sin(start[3])
				matrix[3] = 0.0
				matrix[4] = start[1]
				
				matrix[5] = math.sin(start[3])
				matrix[6] = math.cos(start[3])
				matrix[7] = 0.0
				matrix[8] = start[2]
				
				matrix[9] = 0.0
				matrix[10] = 0.0
				matrix[11] = 0.0
				matrix[12] = 0.002	
				for j=1,#vertices[i],3 do --iterate through vertices on robot shape		
					vec = {vertices[i][j], vertices[i][j+1], vertices[i][j+2]}
					vec = simMultiplyVector(matrix, vec)
					vertices[i][j] = vec[1]
					vertices[i][j+1] = vec[2]
					vertices[i][j+2] = vec[3]
				end	
				h=simCreateMeshShape(2,20*math.pi/180,vertices[i],indices[i])
				simSetShapeColor(h,"",sim_colorcomponent_ambient,{0.5,0.5,0.5})
				simSetObjectName(h,names[i])	
			elseif ((names[i] ~= "StartConfig") and (names[i] ~= "GoalConfig")) then				
				h=simCreateMeshShape(2,20*math.pi/180,vertices[i],indices[i])
				if(names[i]:find("Obstacle") ~= nil) then
					simSetShapeColor(h,"",sim_colorcomponent_ambient,{0.4,0.4,0.4})
				else
					simSetShapeColor(h,"",sim_colorcomponent_ambient,{1.0,1.0,1.0})
				end
				simSetObjectName(h,names[i])
			end
        end
    end
	return start
end

function getPort()
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
	return newPortNb
end

function drawLine(pt1, pt2)
	dh = simAddDrawingObject(sim_drawing_lines,2,0,-1,2,nil,nil,nil,{1,0,0})			
	lineData={pt1[1],pt1[2],pt1[3],pt2[1],pt2[2],pt2[3]}
	simAddDrawingObjectItem(dh,lineData)	
	return dh
end
