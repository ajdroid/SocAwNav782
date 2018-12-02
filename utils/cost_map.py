import numpy as np
import matplotlib.pyplot as plt
import pickle
import argparse
import scipy
from scipy.stats import multivariate_normal
from matplotlib import cm
import os

import pdb

def gaussian_2d(pos, mux, muy, sx, sy, corr, height, width, path_size):
    '''
    Parameters
    ==========

    mux, muy, sx, sy, corr : a tensor of shape 1 x numNodes
    Contains x-means, y-means, x-stds, y-stds and correlation

    pos is 

    Returns
    =======

    next_x, next_y : a tensor of shape numNodes
    Contains sampled values from the 2D gaussian
    '''

    o_mux, o_muy, o_sx, o_sy, o_corr = mux[0, :], muy[0, :], sx[0, :], sy[0, :], corr[0, :]

    numNodes = mux.shape[1]
    
    pdf = np.zeros((height, width))
    
    # pdb.set_trace()

    for node in range(numNodes):
        mean = [o_mux[node], o_muy[node]]
        cov = [[o_sx[node]*o_sx[node], o_corr[node]*o_sx[node]*o_sy[node]], [o_corr[node]*o_sx[node]*o_sy[node], o_sy[node]*o_sy[node]]]
        
        curr_pos = np.copy(pos)

        curr_pos[:,:,0] = np.clip(curr_pos[:,:,0] + o_mux[node], 0, width-1)
        curr_pos[:,:,1] = np.clip(curr_pos[:,:,1] + o_muy[node], 0, height-1)

        rv =  multivariate_normal(mean, cov)

        pdf[curr_pos[:,:,1].astype(np.int), curr_pos[:,:,0].astype(np.int)] += rv.pdf(curr_pos)

    return pdf


def plot_trajectories(true_trajs, pred_trajs, nodesPresent, obs_length, stats, name, plot_directory, withBackground=False):
    '''
    Parameters
    ==========

    true_trajs : Numpy matrix of shape seq_length x numNodes x 2
    Contains the true trajectories of the nodes

    pred_trajs : Numpy matrix of shape seq_length x numNodes x 2
    Contains the predicted trajectories of the nodes

    nodesPresent : A list of lists, of size seq_length
    Each list contains the nodeIDs present at that time-step

    obs_length : Length of observed trajectory

    name : Name of the plot

    withBackground : Include background or not
    '''

    traj_length, numNodes, _ = true_trajs.shape
    # Initialize figure
    plt.figure()
    

    stats = np.asarray(stats)
        
    
    # Load the background
    im = plt.imread('plot/background.jpg')
    if withBackground:
        implot = plt.imshow(im)

    width_true = im.shape[0]
    height_true = im.shape[1]

    if withBackground:
        width = width_true
        height = height_true
    else:
        width = 1
        height = 1
    
    X = np.linspace(0, height, 100)
    Y = np.linspace(0, width, 100)
    X, Y = np.meshgrid(X, Y)
    pos = np.empty(X.shape + (2,))
    pos[:, :, 0] = X
    pos[:, :, 1] = Y

    
    
    pdf = np.zeros((100,100))
    
    for stat in stats:
        mux, muy, sx, sy, corr = stat[0], stat[1], stat[2], stat[3], stat[4]
        pdf += gaussian_2d(pos, mux*height, muy*width, sx*height, sy*width, corr)
    
    traj_data = {}
    for tstep in range(traj_length):
        pred_pos = pred_trajs[tstep, :]
        true_pos = true_trajs[tstep, :]

        for ped in range(numNodes):
            if ped not in traj_data and tstep < obs_length:
                traj_data[ped] = [[], []]

            if ped in nodesPresent[tstep]:
                traj_data[ped][0].append(true_pos[ped, :])
                traj_data[ped][1].append(pred_pos[ped, :])

    for j in traj_data:
        c = np.random.rand(3,)
        true_traj_ped = traj_data[j][0]  # List of [x,y] elements
        pred_traj_ped = traj_data[j][1]

        true_x = [(p[0])*height for p in true_traj_ped]
        true_y = [(p[1])*width for p in true_traj_ped]
        pred_x = [(p[0])*height for p in pred_traj_ped]
        pred_y = [(p[1])*width for p in pred_traj_ped]

        plt.plot(true_x, true_y, color=c, linestyle='solid')
        plt.plot(pred_x, pred_y, color=c, linestyle='dashed')
        plt.plot(true_x[0], true_y[0], color=c, marker='o')

    if not withBackground:
        plt.ylim((0, 1))
        plt.xlim((0, 1))
    else:
        plt.ylim((0, width))
        plt.xlim((0, height))
    
    
    # plt.show()
    if withBackground:
        plt.savefig('plot_with_background/'+name+'.png')
    else:
        plt.savefig(plot_directory+'/'+name+'.png')
       

    plt.gcf().clear()
    plt.close()
    
    plt.figure()
    implot = plt.imshow(im)
    
    plt.contourf(X, Y, pdf, zdir='z', cmap=cm.viridis, alpha=0.5)
    
    if not withBackground:
        plt.ylim((0, 1))
        plt.xlim((0, 1))
    else:
        plt.ylim((0, width))
        plt.xlim((0, height))
    
    if withBackground:
        plt.savefig('plot_with_background/'+name+'_c.png')
    else:
        plt.savefig(plot_directory+'/'+name+'_c.png')
    
    
    plt.gcf().clear()
    plt.close()

def main():
    parser = argparse.ArgumentParser()

    # Experiments

    parser.add_argument('--test_dataset', type=int, default=0,
                        help='test dataset index')

    # Parse the parameters
    args = parser.parse_args()

    # Save directory
    save_directory = 'save/'
    save_directory += str(args.test_dataset) + '/'
    plot_directory = 'plot/'

    f = open(save_directory+'/results.pkl', 'rb')
    results = pickle.load(f)
    
    f = open(save_directory+'/result_stats.pkl', 'rb')
    result_stats = pickle.load(f)
    
    
    # print "Enter 0 (or) 1 for without/with background"
    # withBackground = int(input())
    withBackground = 1

    for i in range(len(results)):
        print(i)
        name = 'sequence' + str(i)
        plot_trajectories(results[i][0], results[i][1], results[i][2], results[i][3], result_stats[i], name, plot_directory, withBackground)


class CostMap(object):
    def __init__(self, height, width, patch_size, save_directory="../prediction_data", stat_data="result_stats.pkl"):
        result_stats_file = os.path.join(save_directory, stat_data)
        f = open(result_stats_file, 'rb')
        self.result_stats = pickle.load(f)
        self.frame_num = np.asarray([i * 10 for i in range(len(self.result_stats))])
        self.height = height
        self.width = width
        self.patch_size = patch_size

    def get_cost_map(self, frame_num, plot=False):
        idx = (np.abs(self.frame_num - frame_num)).argmin()
        height = self.height
        width = self.width

        X = np.linspace(-(self.patch_size/2)+1, self.patch_size/2, self.patch_size)
        Y = np.linspace(-(self.patch_size/2)+1, self.patch_size/2, self.patch_size)
       
        pdf = np.zeros((height, width))

        X, Y = np.meshgrid(X, Y)
        pos = np.empty(X.shape + (2,))
        pos[:, :, 0] = X
        pos[:, :, 1] = Y

        stats = self.result_stats[idx]

        for stat in stats:
            mux, muy, sx, sy, corr = stat[0], stat[1], stat[2], stat[3], stat[4]
            pdf += gaussian_2d(pos, mux*width, muy*height, sx*width, sy*height, corr, height, width, self.patch_size)

        if plot:
            X = np.linspace(0, width, width)
            Y = np.linspace(0, height, height)
            X, Y = np.meshgrid(X, Y)

            plt.figure()    
            plt.contourf(X, Y, pdf, zdir='z', cmap=cm.viridis, alpha=0.5)
            plt.show()

        return pdf

if __name__ == '__main__':
    cost_map = CostMap(1000, 1200, 100)
    pdf = cost_map.get_cost_map(80, plot=True)
