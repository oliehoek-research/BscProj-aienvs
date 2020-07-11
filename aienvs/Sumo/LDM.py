import os
import sys
import logging
import numpy as np
import string

class ldm():
    '''
    An LDM (Local Dynamic Map) module contains the positions and other state attributes of dynamic objects
    in the simulation (vehicles, possibly also traffic lights)
    and adapts the vehicles in a platoon to change their controls accordingly.
    Usage -- as a module: from LDM import ldm (has to be imported after traci.start )
    Then call ldm.init()

    Public methods: getMapSliceByCorners( bottomLeftCoords, topRightCoords )
    getMapSliceByCenter( self, centerCoords, widthInMeters, heightInMeters )
    '''

    def __init__(self, using_libsumo=True):
        if( using_libsumo ):
            import libsumo as SUMO_client
        else:
            import traci as SUMO_client

        self.SUMO_client = SUMO_client

        if 'SUMO_HOME' in os.environ:
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)
        else:
            raise ValueError("Environment variable SUMO_HOME is not set, "
                             "please declare it (e.g. in ~/.bashrc).")
        # should be added once only, otherwise multiple step listeners are created
        self._lightids={}


    #TODO: Wouter: change all verbose prints to logging
    def init(self, waitingPenalty, new_reward, verbose=0):
        ''' LDM()

        Creates and initializes the Local Dynamic Map
        Call after traci has connected
        '''
        self.__optimize=False #set True to disable non-optimized public functions
        self.netBoundaryMeters=list(self.SUMO_client.simulation.getNetBoundary())
        self.netBoundaryMetersLL=list([self.netBoundaryMeters[0][0]-10, self.netBoundaryMeters[0][1]-10])
        self.netBoundaryMetersUR=list([self.netBoundaryMeters[1][0]+10, self.netBoundaryMeters[1][1]+10])
        self.netBoundaryMeters=list( [tuple(self.netBoundaryMetersLL), tuple(self.netBoundaryMetersUR)] )


        self._verbose=verbose
        self._lightids=self.SUMO_client.trafficlight.getIDList()
        self._subscribeToTrafficLights()
        self._lightstate={}
        self._tlPositions={}
        self._waitingPenalty = waitingPenalty
        self.new_reward = new_reward
        self.subscribedVehs=[]

        self._vehSpeeds = {}

    def start(self, sumoCmd:list, PORT:9001):
        """
        @param sumoCmd the sumo command for the start, list of init arguments
        """
        logging.debug("Sumo command:" + str(sumoCmd))
        # traci needs a port, libsumo doesn't
        if( self.SUMO_client.__name__ == "traci" ):
           self.SUMO_client.start(sumoCmd, port=PORT)
        else:
            self.SUMO_client.start(sumoCmd)



    def step(self):
        '''
        This updates the vehicles' states with information from the simulation
        '''
        try:
            self.SUMO_client.simulationStep()
        except self.SUMO_client.TraCIException as exc:
            logging.error(str(exc) + str(" This is some problem of libsumo, but everything still seems to work correctly"))

        self.subscribedVehs = list(self.SUMO_client.vehicle.getIDList())
        for vehID in self.subscribedVehs:
            self._addVehicleSubscription(vehID)

        self.subscriptionResults={}

        for vehID in self.subscribedVehs:
            subscriptionResult = self.SUMO_client.vehicle.getSubscriptionResults(vehID)
            logging.debug("Subscription result: veh: " + vehID + str(subscriptionResult))
            if( subscriptionResult and vehID in self.SUMO_client.vehicle.getIDList() ):
                self.subscriptionResults.update({vehID:subscriptionResult})
            else:
                self.subscribedVehs.remove(vehID)


        self._resetMap()

        if( len(self.subscriptionResults.keys())>0 ):
            self._updateMapWithVehicles( self._getVehiclePositions(self.subscriptionResults) )

        tlState = {}
        for lightid in self._lightids:
    	    tlState[lightid] = self.SUMO_client.trafficlight.getSubscriptionResults(lightid)

        self._updateTrafficLights(tlState)

        if(self._lightids != None):
            for lightid in self._lightids:
                if(self._tlPositions.get(lightid) != None):
                    self._add_stop_lights(self._lightstate[lightid], list(self._tlPositions.get(lightid)) )
        return True


    def close(self):
        """
        close sumo env
        """
        self.SUMO_client.close()

    def isSimulationFinished(self):
        """
        @return minimum number of vehicles that are still expected to leave the net (id 0x7d)
        """
        return (self.SUMO_client.simulation.getMinExpectedNumber() <= 0)




    def getRewardByCorners(self, bottomLeftCoords, topRightCoords, local_rewards, reward_range, function):

        c0 = bottomLeftCoords[0] + 0.5 * (topRightCoords[0] - bottomLeftCoords[0])
        c1 = bottomLeftCoords[1] + 0.5 * (topRightCoords[1] - bottomLeftCoords[1])
        # print(c0, c1)

        rewards = {}

        for radius in reward_range:
            vehicles = self.subscriptionResults
            filteredVehicles = vehicles.copy()
            if local_rewards and radius is not None:
                for vehID in vehicles:
                    position = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_POSITION)

                    if(position[0] < c0 - radius):
                        filteredVehicles.pop(vehID)
                        continue
                    if(position[0] > c0 + radius):
                        filteredVehicles.pop(vehID)
                        continue
                    if(position[1] < c1 - radius):
                        filteredVehicles.pop(vehID)
                        continue
                    if(position[1] > c1 + radius):
                        filteredVehicles.pop(vehID)
                        continue

            rewards[radius] = self._computeReward(filteredVehicles, function)
        return rewards

    def getRewardByCenter( self, centerCoords, widthInMeters, heightInMeters ):
        vehicles = self.subscriptionResults
        filteredVehicles = vehicles.copy()
        for vehID in vehicles:
            position = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_POSITION)

            if(position[0] < centerCoords[0] - heightInMeters/2.):
                filteredVehicles.pop(vehID)
                continue
            if(position[0] > centerCoords[0] + heightInMeters/2.):
                filteredVehicles.pop(vehID)
                continue
            if(position[1] < centerCoords[1] - widthInMeters/2.):
                filteredVehicles.pop(vehID)
                continue
            if(position[1] > centerCoords[0] + widthInMeters/2.):
                filteredVehicles.pop(vehID)
                continue

        return self._computeReward( filteredVehicles )

    def getMapSliceByCorners( self, bottomLeftCoords, topRightCoords ):
        bottomLeftMatrixCoords = self._coordMetersToArray( bottomLeftCoords )
        topRightMatrixCoords = self._coordMetersToArray( topRightCoords )
        return self._arrayMap[bottomLeftMatrixCoords[0]:(topRightMatrixCoords[0]), bottomLeftMatrixCoords[1]:(topRightMatrixCoords[1])].transpose()[::-1]

    def getMapSliceByCenter( self, centerCoords, widthInMeters, heightInMeters ):
        bottomLeftCoords = (centerCoords[0] - widthInMeters/2., centerCoords[1] - heightInMeters/2.)
        topRightCoords = (centerCoords[0] + widthInMeters/2., centerCoords[1] + heightInMeters/2.)
        return self.getMapSliceByCorners( bottomLeftCoords, topRightCoords )

    def setResolutionInPixelsPerMeter( self, pixelsPerMeterWidth, pixelsPerMeterHeight ):
        self._pixelsPerMeterWidth=pixelsPerMeterWidth
        self._pixelsPerMeterHeight=pixelsPerMeterHeight
        self._initializeArrayMap()

    def setResolutionInMetersPerPixel( self, metersPerPixelWidth, metersPerPixelHeight ):
        self.setResolutionInPixelsPerMeter( 1./metersPerPixelWidth, 1./metersPerPixelHeight )

    def setPositionOfTrafficLights( self, lightsPositions ):
        # compute the positions of light heads automatically in case they are not given explicitly
        for lightID in self._lightids:
            if lightID in lightsPositions:
                self.setPositionOfTrafficHeads( lightID, lightsPositions.get(lightID) )
            else:
                coordinates = []
                for laneID in self.SUMO_client.trafficlight.getControlledLanes(lightID):
                    coordinate = self.SUMO_client.lane.getShape(laneID)[1]
                    coordinates.append(coordinate)
                self.setPositionOfTrafficHeads(lightID, coordinates)

    def setPositionOfTrafficHeads( self, lightID, positionsInMeters ):
        self._tlPositions[lightID] = positionsInMeters

    ######## getting trafficlight info. Maybe move to TrafficLight object #######
    def getTrafficLights(self):
        """
        @return the list[string] of all traffic light ids
        """
        return self._lightids

    def getLightState(self, tlid):
        """
        @param tlid the id of a traffic light
        @return the state of the traffic light with given tlid
        """
        return self._lightstate[tlid]

    ######## getting lane info. Maybe move to Map object #######
    def getControlledLanes(self, lightid:str):
        """
        @param lightid the id of the traffic light
        @return the lanes controlled by the given lightid
        """
        return self.SUMO_client.trafficlight.getControlledLanes(lightid)

    def getLaneMaxSpeed(self, laneid:str):
        """
        @param lane the id of a lane
        @return the maximum speed on the lane
        """
        self.SUMO_client.lane.getMaxSpeed(laneid)

    def getLaneShape(self, laneid:str):
        """
        @param lane the id of a lane
        @return the shape of the lane
        """
        self.SUMO_client.lane.getShape(laneid)

    def getLaneVehicles(self, laneid:str):
        """
        @param lane the id of a lane
        @return the vehicles on this lane
        """
        return self.SUMO_client.lane.getLastStepVehicleIDs(laneid)

    ######## getting vehicle info. Maybe move to Vehicle object #######
    def getVehicles(self):
        """
        @return the list[string] of vehicle ids
        """
        return self.subscriptionResults.keys()

    def getVehicleLane(self, vehicleid:str):
        """
        @param vehicleid the id of the vehicle
        @return  the lane id where the vehicle is at this time
        """
        return self.SUMO_client.vehicle.getLaneID(vehicleid)

    def getVehicleWaitingTime(self,vehicleid:str):
        """
        @param vehicleid the id of the vehicle
        @return  the waiting time of the vehicle
        """
        return self.SUMO_client.vehicle.getWaitingTime(vehicleid)

    def getVehicleCO2Emission(self, vehicleid:str):
        """
        @param vehicleid the id of the vehicle
        @return vehicle co2 emission
        """
        return self.SUMO_client.vehicle.getCO2Emission(vehicleid)

    def getFuelConsumption(self, vehicleid):
        """
        @param vehicleid the id of the vehicle
        @return vehicle fuel consumption
        """
        return self.SUMO_client.vehicle.getFuelConsumption(vehicleid)

    def getSpeed(self, vehicleid):
        """
        @param vehicleid the id of the vehicle
        @return  the current speed of the vehicle
        """
        return self.SUMO_client.vehicle.getSpeed(vehicleid)

    def getVehicleMaxSpeed(self, vehicleid):
        """
        @param vehicleid the id of the vehicle
        @return  the maximum speed of the vehicle
        """
        return self.SUMO_client.vehicle.getMaxSpeed(vehicleid)

    def getVehicleAllowedSpeed(self, vehicleid):
        """
        @param vehicleid the id of the vehicle
        @return  the allowed speed of the vehicle
        """
        return self.subscriptionResults.get(vehicleid).get(self.SUMO_client.constants.VAR_ALLOWED_SPEED)

    def getVehiclePosition(self, vehicleid):
        """
        @param vehicleid the id of the vehicle
        @return  the position of the vehicle, unscaled, as in the sumo map
        """
        return self.SUMO_client.vehicle.getPosition(vehicleid)

    def getStartingTeleportNumber(self) :
        """
        @return unknown
        """
        return self.SUMO_client.simulation.getStartingTeleportNumber()
    ########################## private functions ##############################

    def _subscribeToTrafficLights(self):
        logging.debug("LightID subscriptions" + str(self._lightids))
        for lightid in self._lightids:
            self.SUMO_client.trafficlight.subscribe(lightid, (self.SUMO_client.constants.TL_RED_YELLOW_GREEN_STATE, self.SUMO_client.constants.TL_CURRENT_PHASE))

    def _initializeArrayMap( self ):
        if( self._verbose ):
            print( self.netBoundaryMeters[1] )
            print( self.netBoundaryMeters[0] )

        self._arrayMap=np.zeros( self._coordMetersToArray(tuple(( self.netBoundaryMeters[1][0], self.netBoundaryMeters[1][1] )) ) )

    def _resetMap( self ):
        self._arrayMap = np.zeros( self._arrayMap.shape )

    def _coordMetersToArray( self, *coordsInMeters ):
        arrayX = round( (coordsInMeters[0][0] - self.netBoundaryMeters[0][0]) * self._pixelsPerMeterWidth - 0.5 )
        arrayY = round( (coordsInMeters[0][1] - self.netBoundaryMeters[0][1]) * self._pixelsPerMeterHeight - 0.5 )
        return [max(0, arrayX), max(0, arrayY)]

    def _addVehicleSubscription(self, vehID):
        self.SUMO_client.vehicle.subscribe(vehID, (self.SUMO_client.constants.VAR_POSITION, self.SUMO_client.constants.VAR_SPEED, self.SUMO_client.constants.VAR_ALLOWED_SPEED, self.SUMO_client.constants.VAR_WAITING_TIME ))

    def _updateMapWithVehicles( self, floatingCarData ):
        for vehCoords in floatingCarData:
            vehCoordsInArray=self._coordMetersToArray(vehCoords)
            try:
                self._arrayMap[vehCoordsInArray[0], vehCoordsInArray[1]] = self._arrayMap[vehCoordsInArray[0], vehCoordsInArray[1]] + 1
            except IndexError as error:
                print(error)

    #selects which reward function to use (might be parameterized later)
    def _computeReward(self, vehicles, function):
        if function == "eval":
            return self._computeEvalRewards(vehicles)
        elif function == "elise":
            return self._computeRewardElise(vehicles)
        else:
            return self._computeRewardDefault(vehicles)

    # vehicles are a subset of all subscription results
    def _computeRewardDefault( self, vehicles ):
        result = 0
        if not vehicles:
            logging.debug("No vehicles, returning 0 reward")
            return 0

        for vehID in vehicles:

            if self.new_reward:
                waitingTime = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_WAITING_TIME)
                reward = -min(waitingTime, 1.0)
            else:
                if self._waitingPenalty:
                    waitingTime = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_WAITING_TIME)
                speed = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_SPEED)
                allowedSpeed = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_ALLOWED_SPEED)

                if( self._verbose ):
                    if self._waitingPenalty:
                        print(vehID + " waitingTime " + str(waitingTime) + " speed " + str(speed) + " allowedSpeed " + str(allowedSpeed))
                    else:
                        print(vehID + " speed " + str(speed) + " allowedSpeed " + str(allowedSpeed))

                if self._waitingPenalty:
                    clippedWaitingTime = min(waitingTime, 1.0) #min(waitingTime*0.5, 1.0)
                    clippedDelay = max(0, 1 - speed/allowedSpeed)
                    reward = - 0.5*clippedDelay - 0.5*clippedWaitingTime
                else:
                    clippedDelay = max(0, 1 - speed / allowedSpeed)
                    reward = -clippedDelay

                if( self._verbose ):
                    if self._waitingPenalty:
                        print(vehID + " clippedWaitingTime " + str(clippedWaitingTime) + " clippedDelay " + str(clippedDelay) + " reward " + str(reward))
                    else:
                        print(vehID + " clippedDelay " + str(clippedDelay) + " reward " + str(reward))

            result += reward
        return result

    def _computeRewardElise( self, vehicles ):
        result = 0
        if not vehicles:
            logging.debug("No vehicles, returning 0 reward")
            return 0

        # print("Teleport: ", self.SUMO_client.simulation.getStartingTeleportNumber)

        for tlID in self.getTrafficLights():
            lightFlipPenalty = 0
            if(self.getLightState(tlID) == "ryry" or self.getLightState(tlID) == "yryr"):
                # print("punishing flip")
                lightFlipPenalty = -1
            result += (1.5 * lightFlipPenalty)


        for vehID in vehicles:
            if vehID not in self._vehSpeeds:
                self._vehSpeeds[vehID] = []
            currentSpeed = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_SPEED)
            allowedSpeed = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_ALLOWED_SPEED)
            clippedDelay = -1 * max(0, 1 - currentSpeed / allowedSpeed)
            waitPenalty = 0
            hardBrakesPenalty = 0

            if(self.getVehicleWaitingTime(vehID) == 1):
                waitPenalty = -0.5
            if(self.getVehicleWaitingTime(vehID) > 1):
                waitPenalty = -1


            if(len(self._vehSpeeds[vehID]) > 0):
                lastSpeed = self._vehSpeeds[vehID][len(self._vehSpeeds[vehID]) - 1]
                if(currentSpeed - lastSpeed <= -4.5):
                    hardBrakesPenalty = -1

            result += (0.2 * hardBrakesPenalty) + (0.3 * clippedDelay) + (0.3 * waitPenalty)
            self._vehSpeeds[vehID].append(currentSpeed)
        return result

    def _computeEvalRewards(self, vehicles):

        result = 0
        # if not vehicles:
        #     logging.debug("No vehicles, returning 0 reward")
        #     return 0
        for vehID in vehicles:
            if vehID not in self._vehSpeeds:
                self._vehSpeeds[vehID] = []
            currentSpeedFactor = vehicles.get(vehID).get(self.SUMO_client.constants.VAR_SPEED) / vehicles.get(vehID).get(self.SUMO_client.constants.VAR_ALLOWED_SPEED)
            self._vehSpeeds[vehID].append(currentSpeedFactor)
        avgSpeeds = []
        for key in self._vehSpeeds.keys():
            lst = self._vehSpeeds[key]
            avgSpeeds.append(sum(lst) / len(lst))
        # print("\n Car average speeds: ", avgSpeeds)
        if(len(avgSpeeds) > 0):
            systemAverage = sum(avgSpeeds) / len(avgSpeeds)
        else:
            return 0
        # print("\n System average speed: ", systemAverage)


        return systemAverage

    def _getVehiclePositions( self, subscriptionResults ):
        resultsFormatted=list(subscriptionResults.values())
        positionList = list()

        for vehAttrib in resultsFormatted:
            if(vehAttrib):
                position = (round(vehAttrib[self.SUMO_client.constants.VAR_POSITION][0]), round(vehAttrib[self.SUMO_client.constants.VAR_POSITION][1]))
                if(self._verbose):
                    print("Position " + str(position))
                positionList.append(position)
        return positionList

    def _getLaneEnds( self, subscriptionResults ):
        resultsFormatted=list(subscriptionResults.values())
        positionList = list()

        for vehAttrib in resultsFormatted:
            position = (round(vehAttrib[self.SUMO_client.constants.VAR_POSITION][0]), round(vehAttrib[self.SUMO_client.constants.VAR_POSITION][1]))
            if(self._verbose):
                print("Position " + str(position))
            positionList.append(position)
        return positionList

    def _updateTrafficLights(self, lightupdates):
        """
        update the trafficlights cache
        I guess the lightupdate is according to https://sumo.dlr.de/wiki/TraCI/Traffic_Lights_Value_Retrieval
        """
        for lightid in lightupdates:
            lightstate = lightupdates[lightid][self.SUMO_client.constants.TL_RED_YELLOW_GREEN_STATE]
            if(self._verbose):
                print("Light " + lightid + "=" + lightstate)
            self._lightstate[lightid] = lightstate;


    def _add_stop_lights(self, lights, position):
        """
        Add the right value for the state of the traffic light at the right
        position in the matrix.
        Keyword arguments:
            state_matrix -- the matrix in which the values are stores
            light_color  -- a tuple containing the state of the traffic light
            position     -- a tuple containing the traffic light position
            traci        -- instance of TraCI to communicate with SUMO
        """
        index = 0
        for index in range(len(lights)):
            if lights[index] == 'G' or lights[index] == 'g':
                val = 0.8
            elif lights[index] == 'y':
                val = 0.5
            elif lights[index] == 'r':
                val = 0.2

            arrayPosition = self._coordMetersToArray( position[index] )
            self._arrayMap[arrayPosition[0], arrayPosition[1]] += val
            index += 1

    def setRedYellowGreenState(self, agent:string, state:string ):
        """
        set new state for a traffic  light
        @param agent the agent id
        @param state the new state eg "GrGr"
        """
        self.SUMO_client.trafficlight.setRedYellowGreenState(agent, state)

    def test(self, bottomLeftCoord = (506., 430.), topRightCoord = (516., 500.), centerCoord = (510., 475.), width = 10., height=70. ):
        #mapSlice=str(self.getMapSliceByCorners( bottomLeftCoord, topRightCoord ))
        mapSlice=str(self.getMapSliceByCenter( centerCoord, width, height ))
        #for visualization only
        """
        import colorama
        colorama.init()
        def move_cursor(x,y):
            print ("\x1b[{};{}H".format(y+1,x+1))
        def clear():
            print ("\x1b[2J")
        clear()
        move_cursor(100,100)
        """
        logging.debug(mapSlice)

    def __del__(self):
        """
        close sumo env
        """
        self.SUMO_client.close()
