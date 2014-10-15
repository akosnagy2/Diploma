function setRobotSpeed(v, fi)
	-- Set motors
	v = v * -1;
	if (math.abs(fi) < 0.001) then
		simSetJointTargetVelocity(leftMotor, v / (wheelDiameter / 1000 * math.pi))
		simSetJointTargetVelocity(rightMotor, v / (wheelDiameter / 1000 * math.pi))
	else
		r = axisDistance / math.tan(fi)
		speed = v * (r - wheelDistance / 2000) / (r * wheelDiameter / 1000 * math.pi)
		simSetJointTargetVelocity(leftMotor, speed)
		speed = v * (r + wheelDistance / 2000) / (r * wheelDiameter / 1000 * math.pi)
		simSetJointTargetVelocity(rightMotor, speed)
	end
	
	-- Set steering wheels
	simSetJointPosition(leftSteer, math.atan(axisDistance  / (-wheelDistance / 2 + axisDistance  / math.tan(fi))))
	simSetJointPosition(rightSteer, math.atan(axisDistance / (wheelDistance / 2 + axisDistance  / math.tan(fi))))
	--simSetJointTargetPosition(leftSteer, math.atan(axisDistance / (wheelDistance / 2 - axisDistance / math.tan(fi))))
	--simSetJointTargetPosition(rightSteer, math.atan(axisDistance / (wheelDistance / 2 + axisDistance / math.tan(fi))))
end
