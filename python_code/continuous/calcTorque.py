import numpy as np

def calcTorque(pos, item_positions, v_hat, nOfParticles, nOfItems, particle_radius, torque_radius):

    torque = np.zeros((nOfParticles,1))
    torque_item = np.zeros((nOfParticles,1))
    # calculate torque for each particle (single torque depends on all other particles) 
    actActNeig = {i:[] for i in range(nOfParticles)}
    for i in range(1, nOfParticles):
        # calculate direction vector to every vector ( r_i,i is not a thing tho)
        r = pos[i] - pos 
        r_item = pos[i] - item_positions 
        # calculate the norm of every direction vector
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
        # collect only nearby ones
        actActNeig[i] = [part for part in range(nOfParticles) if rnorms[part] < 10 * particle_radius]
        # we need rhat
        r_hat  = r / rnorms
        r_item_hat  = r_item / rnorms_item
        r_hat[i] = np.zeros(2)
        # dot 'em. dot does not support axis thing so we do it like this 
        dots = np.sum(v_hat[i] * r_hat, axis=1).reshape((nOfParticles,1))
        dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItems,1))
        coefs = dots / rnorms**2 
        coefs_item = dots_item / rnorms_item**2 
        # try repelling them now
        coefs[i] = 0
        # crosses v_i with r_i and does so for all i
        crosses = np.cross(v_hat[i], r_hat).reshape((nOfParticles, 1))
        crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItems, 1))
        particle_torques = coefs * crosses
        particle_torques_item = coefs_item * crosses_item



        particle_torques = np.array([particle_torques[p] for p in range(nOfParticles) if rnorms[p] < torque_radius])
        particle_torques_item = np.array([particle_torques_item[p] for p in range(nOfItems) if rnorms_item[p] < torque_radius])


        torque[i] = np.sum(particle_torques) 
        torque_item[i] = np.sum(particle_torques_item) 

    return torque, torque_item, actActNeig
