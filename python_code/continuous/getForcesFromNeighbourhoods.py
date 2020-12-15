import numpy as np


def calcTorqueRob(pos, robRobNeig, v_hat, nOfRobots, particle_radius):
    torque_rob = np.zeros(nOfRobots)

    for i in range(nOfRobots):

        nOfRobotsInNeigh = len(robRobNeig[i])

        if nOfRobotsInNeigh == 0:
            torque_rob[i] = 0
        else:
            pos_neighbs = np.array([pos[n] for n in robRobNeig[i]]) if nOfRobotsInNeigh > 0 else 0
            r_rob = pos[i] - pos_neighbs
            rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobotsInNeigh,1))
            r_rob_hat  = r_rob / rnorms_rob
            dots_rob = np.sum(v_hat[i] * r_rob_hat, axis=1).reshape((nOfRobotsInNeigh,1))
            # 1.5 just in case they collide
            coefs = dots_rob / (rnorms_rob - 1.5*particle_radius)**2
            # NOTE maybe this should be without the square!
            # TODO try both!
            # NOTE: calcRobField in animateField.py is built on this,
            # if you change here you gotta change there as well
            # if you want consistent fields
            crosses = np.cross(v_hat[i], r_rob_hat).reshape((nOfRobotsInNeigh, 1))
            particle_torques = coefs * crosses
            torque_rob[i] = np.sum(particle_torques)

    return torque_rob


def calcTorqueRob_as_v(v, pos, robRobNeig, v_hat, nOfRobots, particle_radius):
    torque_rob = np.zeros((nOfRobots,2))

    for i in range(nOfRobots):

        nOfRobotsInNeigh = len(robRobNeig[i])

        if nOfRobotsInNeigh == 0:
            torque_rob[i] = np.zeros(2)
        else:
            pos_neighbs = np.array([pos[n] for n in robRobNeig[i]]) if nOfRobotsInNeigh > 0 else 0
            r_rob = pos[i] - pos_neighbs
            rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobotsInNeigh,1))
            r_rob_hat  = r_rob / rnorms_rob
            dots_rob = np.sum(v_hat[i] * r_rob_hat, axis=1).reshape((nOfRobotsInNeigh,1))
            # NOTE maybe this should be without the square!
            # TODO try both!
            # NOTE: calcRobField in animateField.py is built on this,
            # if you change here you gotta change there as well
            # if you want consistent fields
            coefs = dots_rob / (rnorms_rob - particle_radius)**2
            crosses = np.cross(v_hat[i], r_rob_hat).reshape((nOfRobotsInNeigh, 1))
            particle_torques = coefs * crosses
            summ = np.sum(particle_torques)
            torque_rob[i] = v * np.array([np.cos(summ), np.sin(summ)])
    return torque_rob


def calcTorqueItem(pos, robItemNeig, v_hat, nOfRobots):

    torque_item = np.zeros(nOfRobots)
    # calculate torque for each particle based on nbhd
    for i in range(nOfRobots):

        nOfItemsInNeigh = len(robItemNeig[i])

        if nOfItemsInNeigh == 0:
            torque_item[i] = 0
        else:
            # calculate direction vector to things in neighbourhood
            r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
            # calculate the norm of every direction vector
            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
            # collect only nearby ones
            # we need rhat
            r_item_hat  = r_item / rnorms_item
            # dot 'em. dot does not support axis thing so we do it like this 
            dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItemsInNeigh,1))
            # NOTE maybe this should be without the square!
            # TODO try both!
            # NOTE: calcItemField in animateField.py is built on this,
            # if you change here you gotta change there as well
            # if you want consistent fields
            coefs_item = dots_item / rnorms_item**2
            # try repelling them now
            # crosses v_i with r_i and does so for all i
            crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItemsInNeigh, 1))
            particle_torques_item = coefs_item * crosses_item

            torque_item[i] = np.sum(particle_torques_item) 

    return torque_item




def calcTorqueObs(pos, robObsNeig, v_hat, nOfRobots, obstacleRadius):

    torque_obs = np.zeros(nOfRobots)
    # calculate torque for each particle based on nbhd
    for i in range(nOfRobots):
        # if robot state is != 0 then there is no effect
        nOfObssInNeigh = len(robObsNeig[i])

        if nOfObssInNeigh == 0:
            torque_obs[i] = 0
            continue
        # calculate direction vector to things in neighbourhood
        r_obs = pos[i] - np.array(list(map(list, robObsNeig[i])))
        # calculate the norm of every direction vector
        # the robot will never get inside the obstacle so minus it to have reasonable forces
        rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1)) - obstacleRadius
        # collect only nearby ones
        # we need rhat
        r_obs_hat  = r_obs / rnorms_obs
        # dot 'em. dot does not support axis thing so we do it like this 
        dots_obs = np.sum(v_hat[i] * r_obs_hat, axis=1).reshape((nOfObssInNeigh,1))
        # NOTE maybe this should be without the square!
        # TODO try both!
        # NOTE: calcObsField in animateField.py is built on this,
        # if you change here you gotta change there as well
        # if you want consistent fields
        coefs_obs = dots_obs / (rnorms_obs - obstacleRadius)**2 
        #coefs_obs = dots_obs / (rnorms_obs - obstacleRadius - particle_radius)**2 
        # try repelling them now
        # crosses v_i with r_i and does so for all i
        crosses_obs = np.cross(v_hat[i], r_obs_hat).reshape((nOfObssInNeigh, 1))
        particle_torques_obs = coefs_obs * crosses_obs

        torque_obs[i] = np.sum(particle_torques_obs) 

    return torque_obs





def calcTorqueObs_as_v(v, pos, robObsNeig, v_hat, explorers, obstacleRadius, nOfRobots, particle_radius):

    torque_obs = np.zeros((nOfRobots,2))
    # calculate torque for each particle based on nbhd
    for i in explorers:
        # if robot state is != 0 then there is no effect
        nOfObssInNeigh = len(robObsNeig[i])

        if nOfObssInNeigh == 0:
            continue
        # calculate direction vector to things in neighbourhood
        r_obs = pos[i] - np.array(list(map(list, robObsNeig[i])))
        # calculate the norm of every direction vector
        # the robot will never get inside the obstacle so minus it to have reasonable forces
        rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1)) - obstacleRadius
        # collect only nearby ones
        # we need rhat
        r_obs_hat  = r_obs / rnorms_obs
        # dot 'em. dot does not support axis thing so we do it like this 
        dots_obs = np.sum(v_hat[i] * r_obs_hat, axis=1).reshape((nOfObssInNeigh,1))
        # NOTE maybe this should be without the square!
        # TODO try both!
        # NOTE: calcObsField in animateField.py is built on this,
        # if you change here you gotta change there as well
        # if you want consistent fields
        coefs_obs = dots_obs / (rnorms_obs - obstacleRadius - 0.5*particle_radius)**2 
        # try repelling them now
        # crosses v_i with r_i and does so for all i
        crosses_obs = np.cross(v_hat[i], r_obs_hat).reshape((nOfObssInNeigh, 1))
        particle_torques_obs = coefs_obs * crosses_obs

        summ = np.sum(particle_torques_obs) 
        torque_obs[i] = v * np.array([np.cos(summ), np.sin(summ)]) 

    return torque_obs



def calcForceRob(v, pos, robRobNeig, nOfRobots, particle_radius):

    force_rob = np.zeros((nOfRobots,2))
    for i in range(nOfRobots):
        nOfRobotsInNeigh = len(robRobNeig[i])

        if nOfRobotsInNeigh == 0:
            force_rob[i] = np.zeros(2)
        else:
            pos_neighbs = np.array([pos[n] for n in robRobNeig[i]]) if nOfRobotsInNeigh > 0 else 0
            r_rob = pos[i] - pos_neighbs
            rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobotsInNeigh,1))
            r_rob_hat  = r_rob / rnorms_rob
            #rob_forces = np.array([r_rob_hat[p] / (rnorms_rob[p] - particle_radius)**2 for p in range(nOfRobotsInNeigh)])
            # trying it without the square 'cos it seemed way too small
        # NOTE maybe this should be without the square!
        # TODO try both!
        # NOTE: calcRobField in animateField.py is built on this,
        # if you change here you gotta change there as well
        # if you want consistent fields
        # TRYITNO
            rob_forces = np.array([r_rob_hat[p] / (rnorms_rob[p] - 1.5*particle_radius)**2 for p in range(nOfRobotsInNeigh)])
            force =  np.sum(rob_forces,axis=0) 
            # it works because it's per element
            # TODO rewrite this last thing in vector form for the speeeed
            force_strength = np.linalg.norm(force)
            force_rob[i] = force if  force_strength < v else v * force / force_strength

    return force_rob




def calcForceItem(v, pos, robItemNeig, nOfRobots, particle_radius):

    force_item = np.zeros((nOfRobots,2))
    for i in range(nOfRobots):
        nOfItemsInNeigh = len(robItemNeig[i])
        if nOfItemsInNeigh == 0:
            force_item[i] = np.zeros(2)
        else:
            r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
            r_item_hat  = r_item / rnorms_item
        # NOTE maybe this should be without the square!
        # TODO try both!
        # NOTE: calcItemField in animateField.py is built on this,
        # if you change here you gotta change there as well
        # if you want consistent fields
            item_forces = np.array([r_item_hat[p] / (rnorms_item[p])**2 for p in range(nOfItemsInNeigh)])
            force =  np.sum(item_forces, axis=0) 
            # TODO rewrite this last thing in vector form for the speeeed
            force_strength = np.linalg.norm(force)
            force_item[i] = force if  force_strength < v else v * force / force_strength

    return force_item




def calcForceObs(v, pos, robObsNeig, nOfRobots, particle_radius, obstacleRadius):

    force_obs = np.zeros((nOfRobots,2))
    for i in range(nOfRobots):
        nOfObssInNeigh = len(robObsNeig[i])
        if nOfObssInNeigh == 0:
            force_obs[i] = np.zeros(2)
        else:
            r_obs = pos[i] - np.array(list(map(list, robObsNeig[i])))
            rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1))
            r_obs_hat  = r_obs / rnorms_obs
            # NOTE maybe this should be without the square!
            # TODO try both!
            # NOTE: calcObsField in animateField.py is built on this,
            # if you change here you gotta change there as well
            # if you want consistent fields
            obs_forces = np.array([r_obs_hat[p] / (rnorms_obs[p] - obstacleRadius)**2 for p in range(nOfObssInNeigh)])
            #obs_forces = np.array([r_obs_hat[p] / (rnorms_obs[p] - obstacleRadius - particle_radius)**2 for p in range(nOfObssInNeigh)])
            force =  np.sum(obs_forces, axis=0) 
            # TODO rewrite this last thing in vector form for the speeeed
            force_strength = np.abs(np.linalg.norm(force))
            force_obs[i] = force if  force_strength < v else v * force / force_strength

    return force_obs



def calcForceObsClusters(v, pos, robObsNeig, obstacleClusters, nOfRobots, particle_radius, obstacleRadius):

    force_obs = np.zeros((nOfRobots,2))
    for i in range(nOfRobots):
        nOfObssInNeigh = len(robObsNeig[i])
        if nOfObssInNeigh == 0:
            continue
        else:
            # go through each cluster
            # and blacklist checked obstacles as you go along

# TODO FIX THIS THING THEN TRY AGAIN

            blackList = set()
            clusterForce = []
            for obstacle in robObsNeig[i]:
                clusterAsSet = obstacleClusters[obstacle].difference(blackList)
                nInCluster = len(clusterAsSet)
                if nInCluster == 0:
                    continue
                # TODO FINISH THIS
                # don't calc for same object twice
                blackList.update(obstacleClusters[obstacle])
                clusterAsList = np.array(list(map(list, clusterAsSet)))
                # calc normally for every force in cluster
                r_obs = pos[i] - clusterAsList
                rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nInCluster,1))
                r_obs_hat  = r_obs / rnorms_obs
                cluster_forces = np.array([r_obs_hat[p] / (rnorms_obs[p] - obstacleRadius )**2 for p in range(nInCluster)])
                # now combine that into a single vector
                cluster_force = np.sum(cluster_forces, axis=0)
                force_strength = np.linalg.norm(cluster_force)
                # get it's direction
                cluster_force_hat = cluster_force / force_strength
                # but pretend the strength is the average of the cluster
                # yes that doesn't really make too much sense but this is engineering not physics,
                # i get to do what i want even if it doesnt reaaally make sense
                r_cluster_avg = np.sum(rnorms_obs) / nInCluster
                cluster_force_coef = r_cluster_avg ** 2
                force = cluster_force_coef * cluster_force_hat

                # try with this first
                clusterForce.append(force if cluster_force_coef < v else v * cluster_force_hat)
                
            final = np.sum(np.array(clusterForce), axis=0)
            # try this laterr
            final_norm = np.linalg.norm(final)
            force_obs[i] = final if final_norm < 0.9*v else v*0.9 * final / final_norm
            force_obs[i] = final
#            r_obs = pos[i] - np.array(list(map(list, robObsNeig[i])))
#            rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1))
#            r_obs_hat  = r_obs / rnorms_obs
#            # NOTE maybe this should be without the square!
#            # TODO try both!
#            # NOTE: calcObsField in animateField.py is built on this,
#            # if you change here you gotta change there as well
#            # if you want consistent fields
#            obs_forces = np.array([r_obs_hat[p] / (rnorms_obs[p] - obstacleRadius)**2 for p in range(nOfObssInNeigh)])
#            force =  np.sum(obs_forces, axis=0) 
#            # TODO rewrite this last thing in vector form for the speeeed
#            # dude this shit is wrong. you're changing the direction here! christ...
#            force_strength = np.linalg.norm(force)
#            force_obs[i] = force if  force_strength< v else v * force / force_strength

#    print(force_obs)
    return force_obs

