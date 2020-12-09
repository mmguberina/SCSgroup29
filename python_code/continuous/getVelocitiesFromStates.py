import numpy as np
from scipy.stats import norm

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

# TODO write this
# can be done in different ways, all relevant of which are inn the
# sample old code folders under some name
# not doing it now as it's easy and i have better things to tend to rn
def swimmersBrownianStyle(fi, v_hat, swimmers, deviation):
    nOfSwimmers = len(swimmers)
    # you need to fix 
    rand = norm.rvs(size=nOfSwimmers, scale=deviation)

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
# --> that's all handeled in the main loop because this is very very much state related
def findAndInitStuck(x, y, timestep, nOfRobots, robot_states, nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance):
    # key: robot_id, value: remaining steps to get unstuck
    newStuckRobots = {}
    # TODO [optimization] do this completely in vector form 
    for i in range(nOfRobots):
        # don't refresh those that are already stuck
        if robot_states[i] == 3:
            continue
        amountMoved = np.sum(np.abs(x[i, timestep - stuckThresholdTime:timestep] - x[i,timestep])) \
                    + np.sum(np.abs(y[i, timestep - stuckThresholdTime:timestep] - y[i,timestep]))
        if amountMoved < stuckThresholdDistance:
            # remember previous state
            newStuckRobots[i] = [robot_states[i], nOfUnstuckingSteps]
            print(newStuckRobots)
    return newStuckRobots



# TODO TEST this thing
# this works in the following manner:
#   instead of generating a new direction for every timestep
#   you stick to one direction for some amount of steps
#   these amount of steps randomly decided on 
#   by sampling from a heavy-tailed distribution
#   making this work will require setting up a new state which denotes that a robot
#   is following the prescribed trajectory
# long walks will be broken if items are encoutered and so on
# of course that is something that will be optimized as well
# NOTE the distribution you want for this is called the cauchy distribution
# (it's heavy tailed but has nice analytical properties (thus fast compute too))
# --> check its wiki page to learn more
# TODO learn how these things are parametrized (check papers) and then do a thing
def generateLevyFlightSteps(v, swimmers, gridSize):
    nOfSwimmers = len(swimmers)
    newLevySwimmers = {}

    # this will sometimes give huge ass values, 
    # it's centered at 0
    # and now it needs to be turned into directions and n of timesteps
    # to normalize somehow, take this to be mean distance and then
    # make all smaller than v equal to v, and all bigger than 
    # gridSize to be equal to gridSize (that's given by robot design)
    rand = np.random.standard_cauchy(nOfSwimmers)
    fi = np.random.random(nOfSwimmers) * 2*np.pi
    v_hat = np.hstack(( np.cos(fi).reshape((nOfSwimmers,1)), 
            np.sin(fi).reshape((nOfSwimmers,1))))

    ind = 0
    nsOfTimesteps = np.abs(np.fix(rand / v)) + 1
    for i in swimmers:
        # make this less crude if possible
        newLevySwimmers[i] = [v_hat[ind], nsOfTimesteps[ind]]
        ind += 1
    return newLevySwimmers


def levySwim(levySwimmers, v_hat):
    for i in levySwimmers:
        v_hat[i] = levySwimmers[i][0]


# now monitor this the same way you monitor unstuckers in the main loop
# (it's mostly the same thing really)
# down the line have choose between levy and brownian based 
# whatever criteria makes sense (count obstacles and items 
# for a limited time frame and similar schemes). 
# then have an ann pick between them (or whatever else)
# then evolve the ann to choose based on the given input so that 
# the goal is maximized (area covered / items collected / whatever)

