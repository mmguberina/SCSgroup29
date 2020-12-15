import numpy as np
from scipy.stats import norm
from getForcesFromNeighbourhoods import *
from animate import *
from realismFunctions import *
from getVelocitiesFromStates import *

def runSim(x, y, item_positions_set, delivery_station, N, nOfRobots, gridSize,  robot_statesPerTime, # sim params
                   v, particle_radius, torque_radius, obstacles,obstacleRadius,         # environment robot physical params 
                   walkType, ni, deviation,                                         # random walk params
                   T0, FR0, FI0, FO0, TR0, TO0,                                                   # artificial potential field parameters 
                   nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance       # unstucking parameters
                   ):

    nOfCollectedItemsPerTime = [[0,0]]
    item_positions_list = np.array(list(item_positions_set))
    item_positions_listPerTime = [[0, item_positions_list]]
    nOfItems = len(item_positions_set)
    nOfStartingItems = nOfItems
    nOfDeliveredItems = 0
    fi = np.random.random(nOfRobots) * 2*np.pi

    # 0 is search, 1 is delivering, 2 is going to pick up a spotted item, 3 if they are unstucking themselves
    # 4 is levy flight (for us that's going in one direction for some amount of time)
    robot_states = np.zeros(nOfRobots)
    robot_storage = {rob:[] for rob in range(nOfRobots)}

    # this deals with state 3 and has to be inited before all else
    unstuckers = {}
    # this deals with state 4
    levySwimmers = {}


    for step in range(N):
        # TODO make 'em all go home after N/2

        ####################################################################################
        # calculate movemenent stuff
        ####################################################################################

        pos = np.hstack((x[:, step].reshape((nOfRobots,1)), 
                         y[:, step].reshape((nOfRobots,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
#        v_hat = np.hstack((np.cos(fi) .reshape((nOfRobots,1)), 
#                            np.sin(fi).reshape((nOfRobots,1))))

        # get nbhds
        robRobNeig, robItemNeig, robObsNeig = getNeighbourhoods(pos, item_positions_list, 
                nOfRobots, nOfItems, particle_radius, torque_radius, obstacles, obstacleRadius)



        ####################################################################################
        # deal with items and states
        ####################################################################################
        # fill this accordingly
        v_hat = np.zeros((nOfRobots, 2))
        # TODO you are (correctly) calculating torques only when you're in 0 state!!!!!!!!!!!!!
        # this needs to be done so that torque calculating functions 
        # output appropriate x and y changes like everything else!
        # or have different functions depending on the state!!
        # TODO that is totally wrong!!!!!!!

        # handle all states here
        # do all calculations in separate functions
        explorers, returners, itemPickers = separateByState(robot_states, nOfRobots)


        # state 0 - active swimming (this is overwritten if it's some other state)
        # or handle state 4 - levy flight (going for some time in one direction)
        if walkType == 'activeSwimming':
            activeSwimmersStyleRW(fi, ni, v_hat, explorers)

# TODO they need to be able to switch from state 4 which they don't do right now
        if walkType == 'levyFlight':
            # handle state 3 - unstucking
            newLevySwimmers = generateLevyFlightSteps(v, explorers, gridSize)
            levySwimmers.update(newLevySwimmers)
            for robo in newLevySwimmers:
                robot_states[robo] = 4

            doneWithLevySwimming = set()
            for robo in levySwimmers:
                levySwimmers[robo][-1] -= 1
                if levySwimmers[robo][-1] == 0:
                    robot_states[robo] = 0
                    # can't remove dict item when iterating over the dict
                    #unstuckers.pop(robo)
                    doneWithLevySwimming.add(robo)
            for robo in doneWithLevySwimming:
                levySwimmers.pop(robo)

            levySwim(levySwimmers, v_hat)


        if walkType == 'brownianMotion':
            swimmersBrownianStyle(fi, v_hat, explorers, deviation)


        # handle state 1 - delivering to station
        v_hats2DelSt = v_hat2DeliveryStationFromState(pos, delivery_station, particle_radius, robot_states, returners)
        for robo in v_hats2DelSt:
            isDone = (v_hats2DelSt[robo] == 0).all()
            # if done, drop off the item and go back to roaming
            if isDone:
                nOfCollectedItemsPerTime.append([step, nOfCollectedItemsPerTime[-1][1] + len(robot_storage[robo])])
                robot_storage[robo].clear()
                robot_states[robo] = 0
            else:
                v_hat[robo] = v_hats2DelSt[robo]

        # handle state 2 - picking up visible item
        newItemPickers = explorers2ItemPickers(robItemNeig, explorers)
        newItemPickers_fromLevy = explorers2ItemPickers(robItemNeig, levySwimmers)
        # since levy flight persists over new timesteps, they need to be deleted from there
        # if they switched state to going to object
        # NOTE this might not actually be what you want to happen, but we'll think about that later
        for newItemPicker_fromLevy in newItemPickers_fromLevy:
            levySwimmers.pop(newItemPicker_fromLevy)

        newItemPickers.update(newItemPickers_fromLevy)
        for robo in newItemPickers:
            robot_states[robo] = 2

        robsWithNearItems = v_hat2NearItem(pos, robItemNeig, particle_radius, itemPickers)
#        # returns either vector (ndarray) to item or tuple denoting item position if it has been picked up
        for robo in robsWithNearItems:
            if type(robsWithNearItems[robo]) == tuple:
                # do this if it failed to procure the item by losing sight of it
                if robsWithNearItems[robo] == (-1,-1):
                    robot_states[robo] = 0
                    continue
                robot_storage[robo].append(robsWithNearItems[robo])
                item_positions_set.remove(robsWithNearItems[robo])
                nOfItems -= 1

                # put it in going back mode
                robot_states[robo] = 1
                nOfDeliveredItems += 1

                if nOfDeliveredItems == nOfStartingItems:
                    print("done")
                    return x, y, nOfCollectedItemsPerTime, item_positions_listPerTime

                item_positions_list = np.array(list(item_positions_set))
                item_positions_listPerTime.append([step, item_positions_list])
            else:
                v_hat[robo] = robsWithNearItems[robo]

        # handle state 3 - unstucking
        if step > stuckThresholdTime:
            newStuckRobots = findAndInitStuck(x, y, step, nOfRobots, robot_states, nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance)
            unstuckers.update(newStuckRobots)
            for robo in newStuckRobots:
                robot_states[robo] = 3

            doneWithUnstucking = set()
            for robo in unstuckers:
                unstuckers[robo][1] -= 1
                if unstuckers[robo][1] == 0:
                    robot_states[robo] = unstuckers[robo][0]
                    # can't remove dict item when iterating over the dict
                    #unstuckers.pop(robo)
                    doneWithUnstucking.add(robo)
            for robo in doneWithUnstucking:
                unstuckers.pop(robo)

            swimmersBrownianStyle(fi, v_hat, unstuckers, deviation)

        #### override whatever if they are going away from the boundary
        overstepers = mapBoundaryEnforcing(pos, gridSize, delivery_station, nOfRobots)
        for robo in overstepers:
            v_hat[robo] = overstepers[robo]
        if walkType == 'activeSwimming' or 'brownianMotion':
            for robo in overstepers:
                fi[robo] = 2*np.pi * np.random.random()

        if walkType == 'levyFlight':
            for robo in overstepers:
                if robo in levySwimmers:
                    randAngle = 2*np.pi * np.random.random()
                    levySwimmers[robo][0] = np.array([np.cos(randAngle), np.sin(randAngle)])



        ####################################################################################
        # superimpose forces
        ####################################################################################

        # TODO check signs, strengths and other stuff
        force_rob = FR0 * calcForceRob(v, pos, robRobNeig, nOfRobots, particle_radius)
        #torque_rob = TR0 * calcTorqueRob_as_v(v, pos, robRobNeig, v_hat, nOfRobots, particle_radius)

        #force_item = FI0 * calcForceItem(v, pos, robItemNeig, nOfRobots, particle_radius)

        #force_obs = FO0 * calcForceObs(v, pos, robObsNeig,  nOfRobots, particle_radius, obstacleRadius)
        force_obs = FO0 * calcForceObs(v, pos, robObsNeig, nOfRobots, particle_radius, obstacleRadius)

        # TODO put the coefficients in the force fuctions
        # and make them < v! (otherwise they will "discontinuously" jump)

#        torque_obs = TO0 * calcTorqueObs_as_v(v, pos, robObsNeig, v_hat, explorers, obstacleRadius, nOfRobots, particle_radius)

        # TODO TODO TODO CHECK WHETHER THIS MAKES SENSE
        #fi = fi - torque_obs

        ####################################################################################
        # perform position updates 
        ####################################################################################

# TODO make should be chosen by some nice ifs because you will be tweaking it a lot
        x[:, step+1] = x[:,step] +  v * v_hat[:,0]
        x[:, step+1] = x[:, step+1] + force_rob[:,0] 
        #x[:, step+1] = x[:, step+1] + torque_rob[:,0] 
        #x[:, step+1] = x[:, step+1] - force_item[:,0] 
        x[:, step+1] = x[:, step+1] + force_obs[:,0] 
#        x[:, step+1] = x[:, step+1] + torque_obs[:,0] 
        x[:, step+1] = x[:, step+1] #% gridSize

        y[:, step+1] = y[:,step] +  v * v_hat[:,1] 
        y[:, step+1] = y[:,step+1] + force_rob[:,1] 
        #y[:, step+1] = y[:, step+1] + torque_rob[:,1] 
        #y[:, step+1] = y[:,step+1] - force_item[:,1] 
        y[:, step+1] = y[:,step+1] + force_obs[:,1] 
#        y[:, step+1] = y[:,step+1] + torque_obs[:,1] 
        y[:, step+1] = y[:,step+1] #% gridSize

        
        # we only need to exclude robots
        pos = np.hstack((x[:, step+1].reshape((nOfRobots,1)), 
                         y[:, step+1].reshape((nOfRobots,1))))


        # here we'll pass by reference (but in the pytonic way)
        # esentially this means that volume exclusion will update the x and y array
        # which exists as the code is running, i.e. they won't be copied when entering to 
        # the function
        # think of this function as simply having the code put in another place, not
        # really a function with inputs and outputs
        volumeExclusion(x, y, pos, step, robRobNeig, robObsNeig, particle_radius, nOfRobots, obstacleRadius)
        # this will be used for interpretable animation
        robot_statesPerTime[:,step] = robot_states 
    return x, y, nOfCollectedItemsPerTime, item_positions_listPerTime
