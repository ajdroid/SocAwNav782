import numpy as np
import search

speed =10
sizeX = 200
sizeY = 200
goalX = 199
goalY = 199
startX = 0
startY = 0

input_preds = np.zeros((3, 200, 200))
preds = np.array([0, 10, 20])

#search.test_main(preds, input_preds)
plan = search.graphSearch(speed, startX, startY, goalX, goalY, input_preds, preds)
print(plan)
