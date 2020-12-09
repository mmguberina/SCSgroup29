import numpy as np

# state 0 : explorers
# roam randomly - forces are superimposed on this
def activeSwimmersStyleRW(fi, ni, v_hat, swimmers):
    nOfSwimmers = len(swimmers)
    rand = (np.random.random(nOfSwimmers) - 0.5) * ni

    ind = 0
    for i in swimmers:
        fi[i] = fi[i] + rand[ind]
        v_hat[i] = np.array( [ np.cos(fi[i]), np.sin(fi[i]) ] )
        ind += 1


# state 1 : returners going back to delivery station
# if you arrive to the delivery station, signal to main program thay you are done
#   --> this is done by sending [0,0] as new speed
def v_hat2DeliveryStationFromState(pos, delivery_station, particle_radius, robot_states, returners):
    v_hat2DeliveryStation = {}
    for i in returners:
        rToDelivery = delivery_station - pos[i]
        rnorms = np.linalg.norm(rToDelivery)
        isDone = rnorms < particle_radius
        v_hat = rToDelivery / rnorms
        
        # zeros means it's done
        v_hat2DeliveryStation[i] = np.zeros(2) if isDone else v_hat
    return v_hat2DeliveryStation




# check transition from state 0 to state 2:
#   --> you can enter from 0 to 2 if you're an explorer who found an item 
def explorers2ItemPickers(robItemNeig, explorers):
    newItemPickers = set()
    for i in explorers:
        nOfItemsInNeigh = len(robItemNeig[i])
        if  nOfItemsInNeigh == 0:
            continue
        else:
            newItemPickers.add(i)
    return newItemPickers

# state2 : item pickers getting to closest item
# go to the closest found item if you're in state 2
# you can transition to state 1 from here if you managed to pick the item you found
#  --> this is done by sending the tuple which donotes the pick-up items position to the main program 
# NOTE you can get pushed and lose sight of the item! thus if you lose it report that and go to state 0
def v_hat2NearItem(pos, robItemNeig, particle_radius, itemPickers):
    robsWithNearItems = {}

    for i in itemPickers:
        nOfItemsInNeigh = len(robItemNeig[i])
        if nOfItemsInNeigh == 0:
            # signal like this if you lose sight of item
            robsWithNearItems[i] = (-1,-1)
            continue
        nearItemsList = np.array(list(map(list, robItemNeig[i])))
        r_item = nearItemsList - pos[i] 
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,))
        # sort by index to get the one you want
        closest = np.argsort(rnorms_item)[0]
        r_closest_hat  = r_item[closest] / rnorms_item[closest]
        isDone = rnorms_item[closest] < particle_radius
        v_hat2Item = r_closest_hat

        # if done give me the item position, else give me the vector pointing towards the item
        robsWithNearItems[i] = tuple(nearItemsList[closest]) if isDone else v_hat2Item
    return robsWithNearItems


# state 3 : unstucking aka going random but remembering last state in which you need to go back to
# stucks need to go randomly with high randomness for some time interval
# TODO debug this thing
def findAndInitStuck(x, y, timestep, nOfRobots, robot_states, nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance):
    # key: robot_id, value: remaining steps to get unstuck
    newStuckRobots = {}
    # TODO [optimization] do this completely in vector form 
    for i in range(nOfRobots):
        # don't refresh those that are already stuck
        if robot_states[i] == 3:
            continue
        amountMoved = np.sum(np.abs(x[i, timestep - stuckThresholdTime:timestep])) \
                    + np.sum(np.abs(y[i, timestep - stuckThresholdTime:timestep]))
        if amountMoved < stuckThresholdDistance:
            # remember previous state
            newStuckRobots[i] = [robot_states[i], nOfUnstuckingSteps]
            print(newStuckRobots)
    return newStuckRobots



# TODO write this thing
def generateLevyFlightSteps(x, y, nOfRobots, robot_states, nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance):
    # key: robot_id, value: remaining steps to get unstuck
    stuckRobots= {}
    # TODO do this completely in vector form 
    for i in range(nOfRobots):
        # don't refresh those that are already stuck
        if robot_states[i] == 3:
            continue
        amountMoved = np.sum(np.abs(x[i, -stuckThresholdTime:])) + np.sum(np.abs(y[i, -stuckThresholdTime:]))
        if amountMoved < stuckThresholdDistance:
            # remember previous state
            stuckRobots[i] = [robot_states[i], nOfUnstuckingSteps]
            robot_states[i] = 3

    return stuckRobots
