#wip

from utm import utmconv
import csv
import matplotlib.pyplot as plt
import numpy as np
path = "/home/daniele/Documents/introduction_to_drone_technology/Module 7 - UAS positioning and navigation/miao.csv"
log = csv.reader(open(path, "r"), delimiter=",")
uc = utmconv()

MAX_SPEED = 1.5 #m/s
