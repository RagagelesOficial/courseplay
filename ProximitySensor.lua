--[[
This file is part of Courseplay (https://github.com/Courseplay/courseplay)
Copyright (C) 2020 Peter Vaiko

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
]]--

---@class ProximitySensor
ProximitySensor = CpObject()

function ProximitySensor:init(node, yRotationDeg, range, height)
    self.node = node
    self.yRotation = math.rad(yRotationDeg)
    self.lx, self.lz = MathUtil.getDirectionFromYRotation(self.yRotation)
    self.range = math.min(range, 3 / math.cos((math.pi / 2 - math.abs(self.yRotation))))
    self.dx, self.dz = self.lx * self.range, self.lz * self.range
    self.height = height or 0
    self.lastUpdateLoopIndex = 0
    self.enabled = true
end

function ProximitySensor:enable()
    self.enabled = true
end

function ProximitySensor:disable()
    self.enabled = false
end

function ProximitySensor:update()
    -- already updated in this loop, no need to raycast again
    if g_updateLoopIndex == self.lastUpdateLoopIndex then return end
    self.lastUpdateLoopIndex = g_updateLoopIndex
    local x, y, z = getWorldTranslation(self.node)
    -- get the terrain height at the end oef the raycast line
    local tx, _, tz = localToWorld(self.node, self.dx, 0, self.dz)
    local y2 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, tx, 0, tz)
    -- make sure the raycast line is parallel with the ground
    local ny = (y2 - y) / self.range
    local nx, _, nz = localDirectionToWorld(self.node, self.lx, 0, self.lz)
    self.distanceOfClosestObject = math.huge
    self.objectId = nil
    if self.enabled then
        raycastClosest(x, y + self.height, z, nx, ny, nz, 'raycastCallback', self.range, self, bitOR(AIVehicleUtil.COLLISION_MASK, 2))
    end
    if courseplay.debugChannels[12] and self.distanceOfClosestObject <= self.range then
        local green = self.distanceOfClosestObject / self.range
        local red = 1 - green
        cpDebug:drawLine(x, y + self.height, z, red, green, 0, self.closestObjectX, self.closestObjectY, self.closestObjectZ)
    end
end

function ProximitySensor:raycastCallback(objectId, x, y, z, distance)
    self.distanceOfClosestObject = distance
    self.objectId = objectId
    self.closestObjectX, self.closestObjectY, self.closestObjectZ = x, y, z
end

function ProximitySensor:getClosestObjectDistance()
    --self:showDebugInfo()
    return self.distanceOfClosestObject
end

function ProximitySensor:getClosestRootVehicle()
    if self.objectId then
        local object = g_currentMission:getNodeObject(self.objectId)
        if object and object.getRootVehicle then
            return object:getRootVehicle()
        end
    end
end

function ProximitySensor:showDebugInfo()
    local text = string.format('%.1f ', self.distanceOfClosestObject)
    if self.objectId then
        local object = g_currentMission:getNodeObject(self.objectId)
        if object then
            if object.getRootVehicle then
                text = text .. 'vehicle' .. object:getName()
            else
                text = text .. object:getName()
            end
        else
            for key, classId in pairs(ClassIds) do
                if getHasClassId(self.objectId, classId) then
                    text = text .. ' ' .. key
                end
            end
        end
    end
    renderText(0.6, 0.4 + self.yRotation / 10, 0.018, text .. string.format(' %d', math.deg(self.yRotation)))
end

---@class ProximitySensorPack
ProximitySensorPack = CpObject()

-- maximum angle we rotate the sensor pack into the direction the vehicle is turning
ProximitySensorPack.maxRotation = math.rad(30)

---@param name string a name for this sensor, when multiple sensors are attached to the same node, they need
--- a unique name
---@param vehicle table vehicle we attach the sensor to, used only to rotate the sensor with the steering angle
---@param node number node (front or back) to attach the sensor to
---@param range number range of the sensor in meters
---@param height number height relative to the node in meters
---@param directionsDeg table of numbers, list of angles in degrees to emit a ray to find objects, 0 is forward, >0 left, <0 right
function ProximitySensorPack:init(name, vehicle, node, range, height, directionsDeg)
    ---@type ProximitySensor[]
    self.sensors = {}
    self.vehicle = vehicle
    self.range = range
    self.node = getChild(node, name)
    if self.node <= 0 then
        -- node with this name does not yet exist
        -- add a separate node for the proximity sensor (so we can rotate it independently from 'node'
        self.node = courseplay.createNode(name, 0, 0, 0, node)
    end
    -- reset it on the parent node
    setTranslation(self.node, 0, 0, 0)
    setRotation(self.node, 0, 0, 0)
    self.directionsDeg = directionsDeg
    self.speedControlEnabled = true
    self.swerveEnabled = false
    self.rotateWithWheels = true
    for _, deg in ipairs(self.directionsDeg) do
        self.sensors[deg] = ProximitySensor(node, deg, self.range, height)
    end
end

function ProximitySensorPack:getRange()
    return self.range
end

function ProximitySensorPack:callForAllSensors(func, ...)
    for _, deg in ipairs(self.directionsDeg) do
        func(self.sensors[deg], ...)
    end
end

function ProximitySensorPack:disableSpeedControl()
    self.speedControlEnabled = false
end

function ProximitySensorPack:enableSpeedControl()
    self.speedControlEnabled = true
end

--- Should this pack used to control the speed of the vehicle (or just delivers info about proximity)
function ProximitySensorPack:isSpeedControlEnabled()
    return self.speedControlEnabled
end

function ProximitySensorPack:disableSwerve()
    self.swerveEnabled = false
end

function ProximitySensorPack:enableSwerve()
    self.swerveEnabled = true
end

--- Should this pack used to initiate swerving another vehicle?
function ProximitySensorPack:isSwerveEnabled()
    return self.swerveEnabled
end

function ProximitySensorPack:disableRotateWithWheels()
    self.rotateWithWheels = false
end

function ProximitySensorPack:update()

    if self.rotateWithWheels then
        -- rotate the entire pack in the direction we are turning
        local normalizedSteeringAngle = AIDriverUtil.getCurrentNormalizedSteeringAngle(self.vehicle)
        local _, yRot, _ = getRotation(getParent(self.node))
        setRotation(self.node, 0, yRot + normalizedSteeringAngle * ProximitySensorPack.maxRotation, 0)
    end

    self:callForAllSensors(ProximitySensor.update)

    -- show the position of the pack
    if courseplay.debugChannels[12] then
        local x, y, z = getWorldTranslation(self.node)
        local x1, y1, z1 = localToWorld(self.node, 0, 0, 0.5)
        cpDebug:drawLine(x, y, z, 0, 0, 1, x, y + 1, z)
        cpDebug:drawLine(x, y + 1, z, 0, 1, 0, x1, y1 + 1, z1)
    end
end

function ProximitySensorPack:enable()
    self:callForAllSensors(ProximitySensor.enable)
end

function ProximitySensorPack:disable()
    self:callForAllSensors(ProximitySensor.disable)
end


--- @return number, table, number distance of closest object in meters, root vehicle of the closest object, average direction
--- of the obstacle in degrees, > 0 right, < 0 left
function ProximitySensorPack:getClosestObjectDistanceAndRootVehicle(deg)
    -- make sure we have the latest info, the sensors will make sure they only raycast once per loop
    self:update()
    if deg and self.sensors[deg] then
        return self.sensors[deg]:getClosestObjectDistance(), self.sensors[deg]:getClosestRootVehicle(), deg
    else
        local closestDistance = math.huge
        local closestRootVehicle
        -- weighted average over the different direction, weight depends on how close the closest object is
        local totalWeight, totalDegs = 0, 0
        for _, deg in ipairs(self.directionsDeg) do
            local d = self.sensors[deg]:getClosestObjectDistance()
            if d < self.range then
                local weight = (self.range - d) / self.range
                totalWeight = totalWeight + weight
                totalDegs = totalDegs + weight * deg
            end
            if d < closestDistance then
                closestDistance = d
                closestRootVehicle = self.sensors[deg]:getClosestRootVehicle()
            end
        end
        return closestDistance, closestRootVehicle, totalDegs / totalWeight
    end
    return math.huge, nil, deg
end

function ProximitySensorPack:disableRightSide()
    for _, deg in ipairs(self.directionsDeg) do
        if deg <= 0 then
            self.sensors[deg]:disable()
        end
    end
end

function ProximitySensorPack:enableRightSide()
    for _, deg in ipairs(self.directionsDeg) do
        if deg <= 0 then
            self.sensors[deg]:enable()
        end
    end
end

---@class ForwardLookingProximitySensorPack : ProximitySensorPack
ForwardLookingProximitySensorPack = CpObject(ProximitySensorPack)

function ForwardLookingProximitySensorPack:init(vehicle, node, range, height)
    ProximitySensorPack.init(self, 'forward', vehicle, node, range, height, {0, 15, 30, 60, -15, -30, -60})
end


---@class BackwardLookingProximitySensorPack : ProximitySensorPack
BackwardLookingProximitySensorPack = CpObject(ProximitySensorPack)

function BackwardLookingProximitySensorPack:init(vehicle, node, range, height)
    ProximitySensorPack.init(self, 'backward', vehicle, node, range, height, {120, 150, 180, -150, -120})
end