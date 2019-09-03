import networkx as nx
from util import get_subway_graph, draw_subway_graph
G = nx.Graph()
G, lines = get_subway_graph('csv', nx.graph.Graph)

import heapq
def dijkstra(G, origen, destino, infinity=float('inf')):
    
    Q = set(G.nodes())
    
    H = [] 
    dist = {}
    prev = {}
    expanded = 0

    for node in G.nodes():
        if node == origen:
            dist[node] = 0   
            heapq.heappush(H, (0, node))
        else:
            dist[node] = infinity
            heapq.heappush(H, (infinity, node))
        prev[node] = None

    while len(Q) != 0:
       
        parent = heapq.heappop(H)[1]
        
        Q.discard(parent)

        expanded += 1

        if(dist[parent] == infinity): return "Grafo inconexo"
        if(parent == destino): 
            return construct_path(G, prev, destino, expanded, origen, dist)

        for neightbor in G.neighbors(parent):

            if neightbor in Q:
                alt = dist[parent] + G.edge[parent][neightbor]['distance']

                if alt < dist[neightbor]:
                    dist[neightbor] = alt
                    prev[neightbor] = parent
                    heapq.heappush(H, (alt, neightbor))
                        

def construct_path(G, prev, destino, expanded, origen, dist):
      
    path = [] 
    node = prev[destino]
    path.append(destino)
    
    while(prev[node] != None):
        path.append(node)
        node = prev[node]
        
    path.append(origen)
    
    return {
        'path': path[::-1],
        'expanded': expanded,
        'distance': dist[destino]
    }

def dijkstra2(G, origen, destino, penalty=5000000, infinity=float('inf')):
    
    linies = {node:[] for node in G.nodes()}
    for node in G.nodes():
        for neighbor in G.neighbors(node):
            if(G.edge[node][neighbor]['line'] not in linies[node]):
                linies[node].append(G.edge[node][neighbor]['line'])
                
    Q = []
    for node in linies:
        for i in linies[node]:
            Q.append((node, i))
    Q = set(Q)
    
    H = [] 
    dist = {}
    prev = {}
    expanded = 0

    for node in Q:
        if node[0] == origen:
            dist[node] = 0   
            heapq.heappush(H, (0, node))
        else:
            dist[node] = infinity
            heapq.heappush(H, (infinity, node))
        prev[node] = None
    
    while len(Q) != 0:

        parent = heapq.heappop(H)[1]
        
        Q.discard(parent)

        expanded += 1
        
        if(dist[parent] == infinity): return "Grafo inconexo"
        if(parent[0] == destino): 
            return construct_path2(prev, destino, expanded, origen, dist, parent[1])

        for neighbor in G.neighbors(parent[0]):

            neighbor_node = (neighbor,parent[1])

            if neighbor_node in Q and G.edge[neighbor][parent[0]]['line'] == parent[1]:

                alt = dist[parent] + G.edge[parent[0]][neighbor_node[0]]['distance']

                if alt < dist[neighbor_node]:
                    dist[neighbor_node] = alt
                    prev[neighbor_node] = parent
                    heapq.heappush(H, (alt, neighbor_node))

        for i in range(0,len(linies[parent[0]])):

            neighbor_node = (parent[0],linies[parent[0]][i])

            if(neighbor_node[1] != parent[1]):
                if neighbor_node in Q:

                    alt = dist[parent] + penalty
                    if alt < dist[neighbor_node]:
                        dist[neighbor_node] = alt
                        prev[neighbor_node] = parent
                        heapq.heappush(H, (alt, neighbor_node))
                        

def construct_path2(prev, destino, expanded, origen, dist, linia):
    
    path = [] 
    node = prev[(destino, linia)]
    path.append(destino)
    
    #Dentro del while hay que evitar que un nodo salga más de una vez por el path.
    while(prev[node] != None):
        if path[-1] != node[0]:
            path.append(node[0])
        node = prev[node]
        
    path.append(origen)

    
    return {
        'path': path[::-1],
        'expanded': expanded,
        'distance': dist[(destino,linia)]
    }

def dist_mat(G):
    
    result = {}
    
    for node in G.nodes():
        result[node] = dijkstra3(G,node)
    
    return result
    
    
def dijkstra3(G, origen, infinity=float('inf')):
    
    Q = set(G.nodes())
    H = [] 
    dist = {}
    expanded = 0

    for node in G.nodes():
        if node == origen:
            dist[node] = 0   
            heapq.heappush(H, (0, node))
        else:
            dist[node] = infinity
            heapq.heappush(H, (infinity, node))
    
    while len(Q) != 0:
        
        parent = heapq.heappop(H)[1]
        
        Q.discard(parent)

        expanded += 1

        for neightbor in G.neighbors(parent):
            if neightbor in Q:
                alt = dist[parent] + G.edge[parent][neightbor]['distance']

                if alt < dist[neightbor]:
                    dist[neightbor] = alt
                    heapq.heappush(H, (alt, neightbor))
    
    return dist
