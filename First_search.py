import numpy as np

grid = [[0,0,1,0,0,0],
        [0,0,1,0,0,0],
        [0,0,0,0,1,0],
        [0,0,1,1,1,0],
        [0,0,0,0,1,0]]

start = grid[0][0]
goal = grid[4][5]
i=0
j=0
search = [[i,j]]
g_value = []
# search.append([1,2])
# print(not [0,0] in search)
print(search)

G = 0
for iter in range(20):
    flag = 0
    
    for k in range(len(search)):
        search_next = []
        i = search[k][0]
        j = search[k][1]
        
        if i == 4 and j == 5:
            print("G Value is",G)
            flag = 1
            break   
        
        if i+1 <= 4 and grid[i+1][j] != 1 and not [i+1,j] in search:
            search.append([i+1,j])
           
        if j+1 <= 5 and grid[i][j+1] != 1 and not [i,j+1] in search:
            search.append([i,j+1])

        if i-1 >= 0 and grid[i-1][j] != 1 and not [i-1,j] in search :
            search.append([i-1,j])

        if j-1 >= 0 and grid[i][j-1] != 1 and not [i,j-1] in search:
            search.append([i,j-1])
    if flag == 1:
        break

    print(search)
    # search = search_next
    
    G+=1

    
    
    



