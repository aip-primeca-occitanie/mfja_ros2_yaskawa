#! /usr/bin/env python
import numpy as np
import csv
import argparse


def moveJ(qi,qf,tf,Te):
    """Calculate the different angular values.
    
    INPUT: initial articulations values, final articulations values, total time, step time

    OUTPUT: matrix 6x20"""
    l1=np.array(([1,0,0,0]))
    l2=np.array(([0,1,0,0]))
    l3=np.array(([1,tf,tf**2,tf**3]))
    l4=np.array(([0,1,2*tf,3*(tf**2)]))

    A = np.array(l1)
    A = np.vstack((A,l2))
    A = np.vstack((A,l3))
    A = np.vstack((A,l4))
    A = np.linalg.inv(A)

    for j in range (round(tf/Te)) : 
        for i in range (6) :
            Q = np.array(([qi[i]],[0],[qf[i]],[0]))
            C = np.matmul(A,Q)

            a = C[0] + C[1]*Te*j + C[2]*((Te*j)**2) + C[3]*((Te*j)**3)
            if (i == 0) :  
                b = np.array(a)
            else :
                b = np.append(b,a)
        if (j == 0):
            q = np.array(b)
        else :
            q = np.vstack((q,b))
    return q


def create_traj(qi, qf, tf=5, Te=0.1):
    q = moveJ(qi,qf,tf,Te)
    print (q)
    return q


def write_traj_to_file(traj, filename: str):
    with open(filename,'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(traj)):
            csv_writer.writerow(traj[i])


if __name__=="__main__":

    # Get trajectory file path from user input
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename")
    args = parser.parse_args()

    if args.filename is None:
        filename = '../trajectories/trajectory.csv'
    else:
        filename = args.filename

    # Example initial and goal positions
    qi = [-0.23163650929927826, 0.24303115904331207, -0.9070761203765869, -0.04309574142098427, -0.28723666071891785, 0.34613490104675293]
    qf = [-0.139, -0.063, -0.545, -0.139, -1.109, 0.346]
    # qi=[0.4419472813606262, 0.13577520847320557, 1.0137178897857666, -0.7563529014587402, -0.2653222978115082, 0.03390097618103027]
    # qf = np.copy(qi)
    # qf[0] = qf[0]+0.34

    # Compute and write the trajectory
    traj = create_traj(qi, qf)
    write_traj_to_file(traj, filename)

    print("Trajectory written to file :", filename)
