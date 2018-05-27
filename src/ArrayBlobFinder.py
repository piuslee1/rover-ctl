#! /usr/bin/env python
from __future__ import print_function

from random import randint
from math import atan2,pi

occ_loc     = "."
unocc_loc   = "_"
leaf_loc    = "o"
goal_loc = "#"
start_loc = "@"
block_loc = "|"

def grid_neighbors(coord):
    x = coord[0]
    y = coord[1]
    yield (x+1,y)
    yield (x-1,y)
    yield (x,y+1)
    yield (x,y-1)

def grid_index(grid,coord):
    return grid[coord[0]][coord[1]]

def grid_set(grid,coord,value):
    grid[coord[0]][coord[1]] = value

def grid_random(width,height,vals,weight):
    range_vals = len(vals)-1
    if range_vals < 0:
        raise ValueError("No values")
    grid = []
    for i in xrange(0,width):
        grid.append([])
        for j in xrange(0,height):
            val = 0 if randint(0,99) < weight else 1
            grid[i].append(vals[val])
    return grid

def grid_traverse(grid):
    x = 0
    y = 0
    for row in grid:
        for item in row:
            yield (x,y),item
            y = y + 1
        y = 0
        x = x + 1

def grid_random_recursive(width,height,vals,weight,depth,neighbor_weight):
    grid = grid_random(width,height,vals,weight)
    for i in xrange(0,depth):
        updates = set()
        for item,val in grid_traverse(grid):
            if val != vals[1]:
                continue
            for n in grid_neighbors(item):
                if grid_contains(grid,n) and not grid_index(grid,n) == vals[1]:
                    n_val = 0 if randint(0,99) < neighbor_weight else 1
                    updates.add((n,vals[n_val]))
        for update in updates:
            grid_set(grid,update[0],update[1])
    return grid



def grid_contains(grid,coord):
    x = coord[0]
    y = coord[1]
    if x < 0 or x >= len(grid):
        return False
    if y < 0 or y >= len(grid[x]):
        return False
    return True

def find_blob(coords,grid,occ,neighbor_fn,reject_fn=None):
    if reject_fn is None:
        def null(v):
            return False
        reject_fn = null
    new = set([coords])
    blob = set([coords])
    leaves = set()
    while len(new):
        old = new
        new = set()
        for coord in old:
            for neighbor in neighbor_fn(coord):
                if not grid_contains(grid,neighbor):
                    continue
                val = grid_index(grid,neighbor)
                if reject_fn(val):
                    continue
                if not val == occ:
                    leaves.add(neighbor)
                elif neighbor not in blob:
                    blob.add(neighbor)
                    new.add(neighbor)
        old.clear()
    return leaves,blob
 

def grid_print(grid):
    print()
    for row in grid:
        for item in row:
            print(item,end="")
        print()
    print()

def random_index(width,height):
    return (randint(0,width-1),randint(0,height-1))

def get_quadrant(coord,center):
    x = coord[0] - center[0]
    y = coord[1] - center[1]
    if x >= 0 and y > 0:
        return 0
    elif x > 0 and y <= 0:
        return 1
    elif x <= 0 and y < 0:
        return 2
    else:
        return 3

def angle_to_center(coord,center):
    x = coord[0] - center[0]
    y = coord[1] - center[1]
    angle = (atan2(y,x) + 2*pi) % pi
    return angle

def make_clockwise(center):
    def clockwise(c0,c1):
        diff = -(angle_to_center(c0,center) - angle_to_center(c1,center))
        if diff > 0:
            return 1
        elif diff < 0:
            return -1
        else:
            return 0
    return clockwise

def traverse_leaves(leaves,order_fn):
    leaves = list(leaves)
    leaves.sort(order_fn)
    return leaves

def main():
    width = 50
    height = 50
    test = grid_random_recursive(width,height,(unocc_loc,block_loc),98,2,30)
    start = random_index(width,height)
    goal = random_index(width,height)
    while not grid_index(test,start) == unocc_loc:
        start = random_index(width,height)

    print("CHOSE: {}".format(start))
    grid_set(test,goal,goal_loc)
    grid_set(test,start,start_loc)
    # grid_print(test)

    block_reject =lambda v: (v == block_loc)

    explored = set()
    leaves = set()
    while goal not in leaves:
        for leaf in leaves:
            grid_set(test,leaf,occ_loc)        
        leaves,blob = find_blob(start,test,occ_loc,grid_neighbors,block_reject)
        explored.update(blob)

    traverse = grid_random(width,height,(unocc_loc,occ_loc),100)
    index = 0
    clk = make_clockwise(start)
    for leaf in traverse_leaves(leaves,clk):
        # print(angle_to_center(leaf,start))
        grid_set(traverse,leaf,str(index))
        index = index + 1
    grid_print(traverse)

    final = grid_random(width,height,(unocc_loc,occ_loc),100)
    for item in explored:
        grid_set(final,item,occ_loc)
    for item,val in grid_traverse(test):
        if val == block_loc:
            grid_set(final,item,val)

    grid_set(final,goal,goal_loc)
    grid_set(final,start,start_loc)
    # grid_print(final)


if __name__ == "__main__":
    main()



