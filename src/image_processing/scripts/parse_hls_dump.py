#!/usr/bin/env python2
'''Simple parser of HLSDump.txt + graph of the results'''
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import mpl_toolkits.mplot3d
import re
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process a HLSDump file.')
    parser.add_argument('file', type=argparse.FileType('r'), 
                    help='The HLSDump file')
    args = parser.parse_args()
    data = args.file.read()
    args.file.close()
    
    values = re.findall('\[\ ?(\d+) (\d+) (\d+)\]', data)
    hues = []
    lightness = []
    saturations = []
    for value in values:
        [h, l, s] = value
        hues.append(int(h))
        lightness.append(int(l))
        saturations.append(int(s))

    gs = gridspec.GridSpec(2, 2)
    fig = plt.figure()

    graph1 = fig.add_subplot(gs[0, 0])
    graph1.scatter(hues, saturations)
    graph1.set_xlabel("Hues")
    graph1.set_ylabel("Saturations")

    graph2 = fig.add_subplot(gs[0, 1])
    graph2.scatter(hues, lightness)
    graph2.set_xlabel("Hues")
    graph2.set_ylabel("Lightness")

    graph3 = fig.add_subplot(gs[1, :], projection='3d')
    graph3.scatter(hues, saturations, lightness)
    graph3.set_xlabel("Hues")
    graph3.set_ylabel("Saturations")
    graph3.set_zlabel("Lightness")
    graph3.set_xlim3d(0, 0xFF)
    graph3.set_ylim3d(0, 0xFF)
    graph3.set_zlim3d(0, 0xFF)

    fig.show()

    raw_input('Press Enter to exit')
