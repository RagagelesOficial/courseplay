--[[
This file is part of Courseplay (https://github.com/Courseplay/courseplay)
Copyright (C) 2018 Peter Vajko

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
]]

--- A traffic conflict between two vehicles.
--- A conflict is created when the collision boxes of two vehicles overlap.
---@class Conflict
Conflict = CpObject()

-- a collision trigger must be present for at least so many milliseconds before it is considered for a conflict
Conflict.detectionThresholdMilliSec = 0
-- a collision trigger must be cleared for at least so many milliseconds before it is removed from a conflict
Conflict.clearThresholdMilliSecNormal = 1000
-- if this is a head-on conflict, don't clear the conflict while we drive around the other vehicle
Conflict.clearThresholdMilliSecHeadOn = 12000

function Conflict:init(vehicle, otherVehicle, triggerId, d, eta, otherD, otherEta, yRotDiff)
	self.debugChannel = 3
	self.vehicle = vehicle
	self.otherVehicle = otherVehicle
    self.triggers = {}
	-- need to count them ourselves as the triggers table is a hash, not an array
	self.nTriggers = 0
	-- we don't yet know who has the right of way in this conflict
	self.rightOfWayEvaluated = false
	self:onDetected(triggerId, d, eta, otherD, otherEta, yRotDiff)
	self:evaluateRightOfWay()
end

function Conflict:debug(...)
	courseplay.debugVehicle(self.debugChannel, self.vehicle,
			string.format('in conflict with %s: ', nameNum(self.otherVehicle)) .. string.format(...))
end

function Conflict:isVehicleInvolved(vehicle)
	return vehicle == self.vehicle or vehicle == self.otherVehicle
end

function Conflict:isBetween(vehicle, otherVehicle)
	if vehicle == self.vehicle and otherVehicle == self.otherVehicle then
		return true
	elseif vehicle == self.otherVehicle and otherVehicle == self.vehicle then
		return true
	else
		return false
	end
end

function Conflict:isWith(otherVehicle)
	return self.otherVehicle == otherVehicle
end

function Conflict:isCleared()
	return self.nTriggers < 1
end

function Conflict:onDetected(triggerId, d, eta, otherD, otherEta, yRotDiff)
	self.triggers[triggerId] = {d = d, eta = eta, otherD = otherD, otherEta = otherEta,
								yRotDiff = yRotDiff, detectedAt = g_time}
	self:update()
end

function Conflict:onCleared(triggerId)
	if not self.triggers[triggerId] then return end
	self.triggers[triggerId].timeCleared = g_time
	self:update()
end

function Conflict:getClearThresholdMilliSec()
	if self.headOn then
		return Conflict.clearThresholdMilliSecHeadOn
	else
		return Conflict.clearThresholdMilliSecNormal
	end
end

function Conflict:update()
	local minEta = math.huge
	self.nTriggers = 0
	local triggersToRemove = {}
	for id, trigger in pairs(self.triggers) do
		if not trigger.timeCleared and trigger.eta < minEta and g_time - trigger.detectedAt > Conflict.detectionThresholdMilliSec then
			self.closestTrigger = trigger
			minEta = self.closestTrigger.eta
		end
		self.nTriggers = self.nTriggers + 1
		if trigger.timeCleared and g_time - trigger.timeCleared > self:getClearThresholdMilliSec() then
			-- been cleared long ago, mark for removal
			table.insert(triggersToRemove, id)
			self.nTriggers = self.nTriggers - 1
		end
	end
	-- remove cleared triggers
	for _, id in pairs(triggersToRemove) do
		self.triggers[id] = nil
	end
end

function Conflict:getDistance()
	if self.closestTrigger then
		return self.closestTrigger.d, self.closestTrigger.eta
	else
		return math.huge, math.huge
	end
end

function Conflict:getConflictingVehicle()
	return self.otherVehicle
end

function Conflict:__tostring()
	local result = string.format('Traffic conflict: %s <-> %s %d triggers', self.vehicle:getName(), self.otherVehicle:getName(), self.nTriggers)
	if self.closestTrigger then
		result = string.format('%s, closest %.1f m %d sec %.1f° ', result, self.closestTrigger.d or -1, self.closestTrigger.eta or -1,
				self.closestTrigger.yRotDiff and math.deg(self.closestTrigger.yRotDiff) or 0)
	end
	return result
end

--- Decide who has the right of way
function Conflict:evaluateRightOfWay()
	-- already know, never re-evaluate
	self:debug('Evaluating right-of-way, closest conflict %s, evaluated %s', tostring(self), self.rightOfWayEvaluated)
	if self.rightOfWayEvaluated == true then return end
	if self.closestTrigger then
		if math.abs(self.closestTrigger.yRotDiff) < math.rad(45) then
			-- one vehicle is behind the other, hold the one behind, let the one on the front drive...
			if AIDriverUtil.isBehindOtherVehicle(self.vehicle, self.otherVehicle) then
				self.mustYield = true
				self:debug('behind other vehicle, I must yield')
			else
				self.mustYield = false
				self:debug('in front of other vehicle, I have right of way')
			end
		else
			-- vehicles crossing paths, decide on priority
			if self.vehicle.cp.driver:is_a(CombineUnloadAIDriver) and
					self.otherVehicle.cp.driver:is_a(CombineAIDriver) then
				self.mustYield = true
				self:debug('I am unloader, other vehicle is combine, I must yield')
			elseif self.otherVehicle.cp.driver:is_a(CombineUnloadAIDriver) and
					self.vehicle.cp.driver:is_a(CombineAIDriver) then
				self.mustYield = false
				self:debug('I am combine, other vehicle is unloader, I have right of way')
			else
				self.mustYield = false
				self:debug('I detected the conflict first, I have right of way')
			end
			if math.abs(self.closestTrigger.yRotDiff) > math.rad(135) then
				-- head on conflict, hold the vehicle with right of way until the other one recalculates
				self:debug('head on conflict')
				self.headOn = true
			end
		end
		if self.otherVehicle.cp.driver then
			-- tell the other vehicle the result of our decision. Whoever gets here first, makes the decision
			-- about the right of way for both participants of the conflict
			self.otherVehicle.cp.driver:onRightOfWayEvaluated(self.vehicle, not self.mustYield, self.headOn)
		end
	end
end

-- The other vehicle in the conflict just decided who has the right of way
function Conflict:onRightOfWayEvaluated(mustYield, headOn)
	self.mustYield = mustYield
	self.headOn = headOn
	self.rightOfWayEvaluated = true
end

--- TrafficController provides a cooperative collision avoidance facility for all Courseplay driven vehicles.
--

TrafficController = CpObject()
TrafficController.debugChannel = 4

function TrafficController:init()
	---@type Conflict[]
	self.conflicts = {}
	self:debug('initialized')
end

-- This should be called once in an update cycle (globally, not vehicle specific)
function TrafficController:update()
	-- iterate backwards as we'll remove table elements
	for i = #self.conflicts, 1, -1 do
		---@type Conflict
		local conflict = self.conflicts[i]
		conflict:update()
		if conflict:isCleared() then
			self:debug('Conflict cleared: %s', self.conflicts[i])
			--self:notifyVehiclesOnConflictCleared()
			table.remove(self.conflicts, i)
		end
	end
	self:drawDebugInfo()
end

function TrafficController:debug(...)
	courseplay:debug('TrafficController: ' .. string.format(...), self.debugChannel)
end

function TrafficController:drawDebugInfo()
	if not courseplay.debugChannels[self.debugChannel] then return end
	local x, y, size = 0.1, 0.8, 0.012
	for i, conflict in ipairs(self.conflicts) do
		renderText(x, y, size, string.format('%d %s', i, conflict))
		y = y - size * 1.1
	end
end

function TrafficController:onConflictDetected(vehicle, otherVehicle, triggerId, d, eta, otherD, otherEta, yRotDiff)
	for _, conflict in ipairs(self.conflicts) do
		if conflict:isBetween(vehicle, otherVehicle) then
			conflict:onDetected(triggerId, d, eta, otherD, otherEta, yRotDiff)
			return
		end
	end
	-- first conflict for this vehicle pair
	table.insert(self.conflicts, Conflict(vehicle, otherVehicle, triggerId, d, eta, otherD, otherEta, yRotDiff))
	self:debug('Conflict added: %s', self.conflicts[#self.conflicts])
end

function TrafficController:onConflictCleared(vehicle, otherVehicle, triggerId)
	for i, conflict in ipairs(self.conflicts) do
		if conflict:isBetween(vehicle, otherVehicle) then
			conflict:onCleared(vehicle, triggerId)
			return
		end
	end
end

function TrafficController:removeAllConflictsForVehicle(vehicle)
	self:debug('Removing all traffic conflicts for %s', nameNum(vehicle))
	for i = #self.conflicts, 1, -1 do
		---@type Conflict
		local conflict = self.conflicts[i]
		if conflict:isVehicleInvolved(vehicle) then
			table.remove(self.conflicts, i)
		end
	end
end

g_trafficController = TrafficController()

