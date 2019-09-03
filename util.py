import csv
import networkx as nx
import math
import matplotlib.pyplot as plt
import traceback
import random as rd
from black_box import real_d2

NX_GRAPH = "<class 'networkx.classes.graph.Graph'>"
NUM_PRUEBAS = 42

def get_subway_graph(csv_dir, Klass):
    is_personal_graph_class = True
    if str(Klass) == NX_GRAPH:
        is_personal_graph_class = False
    G = Klass()
    # "line","name","colour","stripe"
    lines = {}
    with open(csv_dir+'/lines.csv', 'r') as csvfile:
        creader = csv.reader(csvfile)
        next(creader, None)
        for row in creader:
            lines[int(row[0])] = {"name": row[1], "color": row[2], "stripe": row[3], "line":int(row[0])}

    with open(csv_dir+'/connections.csv', 'r') as csvfile:
        creader = csv.reader(csvfile)
        next(creader, None)

        if is_personal_graph_class:
            for row in creader:
                G.add_edge(int(row[0]), int(row[1]), attr_dict=lines[int(row[2])])
        else:
            for row in creader:
                G.add_edge(int(row[0]), int(row[1]),   name=lines[int(row[2])]['name'],
                                                       color=lines[int(row[2])]['color'],
                                                       stripe=lines[int(row[2])]['stripe'],
                                                       line=lines[int(row[2])]['line'])

    with open(csv_dir+'/stations.csv', 'r') as csvfile:
        creader = csv.reader(csvfile)
        next(creader, None)
        if is_personal_graph_class:
            for row in creader:
                G.node[int(row[0])] = {"latitude": float(row[1]),
                                       "longitude": float(row[2]),
                                       "name": row[3],
                                       "display_name": row[4],
                                       "zone": float(row[5]),
                                       "total_lines": int(row[6]),
                                       "rail": row[7]
                                      }
        else:
            for row in creader:
                G.add_node(int(row[0]),    latitude=float(row[1]),
                                           longitude=float(row[2]),
                                           name=row[3],
                                           display_name=row[4],
                                           zone=float(row[5]),
                                           total_lines=int(row[6]),
                                           rail=row[7])

    for node1, node2 in G.edges():
        norm = math.sqrt(
                (G.node[node1]['longitude'] - G.node[node2]['longitude'])**2 +
                (G.node[node1]['latitude'] - G.node[node2]['latitude'])**2
        )
        if is_personal_graph_class:
            G.edge[node1][node2].update({'distance': norm})
        else:
            G.add_edge(node1, node2, distance=norm)

    return G, lines

def draw_subway_graph(G, lines, figsize=(10,6), show_labels=False):
    plt.figure(figsize=figsize)
    plt.axis('off')
    if str(type(G)) != NX_GRAPH:
        G2 = graph2nx(G)
    else:
        G2 = G
    pos = {x: (G2.node[x]['longitude'], G2.node[x]['latitude']) for x in G2.node.keys()}
    nx.draw_networkx_nodes(G2, 
                           pos, 
                           node_size=1,
                          )
    if show_labels:
        nx.draw_networkx_labels(G2,pos,
                                {x: G2.node[x]['name'] for x in G2.nodes()},font_size=4)

    if str(type(G)) == NX_GRAPH:
        for line in lines.keys():
            nx.draw_networkx_edges(
                G2,
                pos,
                edgelist=[x for x in G2.edges() if G.edges[x[0],x[1]]['line'] == line],
                edge_color="#"+lines[line]['color'],
            )
    else:
        for line in lines.keys():
            nx.draw_networkx_edges(
                G2,
                pos,
                edgelist=[x for x in G2.edges() if G.edge[x[0]][x[1]]['line'] == line],
                edge_color="#"+lines[line]['color'],
            )

    plt.show()


def graph2nx(gr):
    G = nx.Graph()
    for node1 in gr.edge.keys():
        for node2, value in gr.edge[node1].items():
            G.add_edge(node1, node2, **value)

    for node, value in gr.node.items():
        G.add_node(node, **value)
       
    return G

def get_path_distance(nxG,path):
    distance = 0
    for i in range(len(path) - 1):
        distance += (nxG[path[i]][path[i + 1]]['distance'])
    return distance

def dijkstra_tester(exercise_number,graph_class,dijkstra_method,verbose=False):
    try:
        num_prueba = 0
        nxG, lines_nxG = get_subway_graph('csv', nx.graph.Graph)
        G, lines = get_subway_graph('csv',graph_class)
        all_nodes = [x for x in nxG.node]
        aciertos = 0
        fail = False
        res_real = dict()
        while num_prueba < NUM_PRUEBAS:
            orig = rd.choice(all_nodes)
            dest = rd.choice(all_nodes)
            if exercise_number == 1:
                res_real['path'] = nx.dijkstra_path(nxG,orig,dest,'distance')
                res_real['distance'] = get_path_distance(nxG, res_real['path'])
                res = dijkstra_method(G,orig,dest)
            elif exercise_number == 2:
                pena = rd.randint(2, 10000)
                res_real = real_d2(nxG, orig, dest, pena)
                res = dijkstra_method(G, orig, dest, pena)
                print(pena)
            else:
                print("Solo hay tester para los ejercicios 1 y 2.")
                fail = True
                break
            if res_real['path'] == res['path']:
                if res_real['distance'] == res['distance']:
                    if verbose:
                        print("Prueba "+str(num_prueba)+" superada.")
                    aciertos += 1
                else:
                    if verbose:
                        print("Prueba " + str(num_prueba) + " fallida. " \
                                                        "\n\tEl path es el mismo, pero no las distancias: " \
                                                        "\n\t\t Distancia esperada : "+str(res_real['distance'])+"" \
                                                        "\n\t\t Distancia obtenida : "+str(res['distance']))
            else:
                if verbose:
                    print("Prueba " + str(num_prueba) + " fallida. El camino no es correcto: " \
                                                    "\n\tCamino esperado : " + str(res_real['path']) + "" \
                                                    "\n\tCamino obtenido : " + str(res['path']))
            num_prueba += 1
        if not fail:
            if not verbose:
                comentario = ""
            else:
                if aciertos == NUM_PRUEBAS:
                    comentario = "Dijkstra es correcto :D, queda en tus manos que sea eficiente."
                elif aciertos == 0:
                    comentario = "Dijkstra no ha pasado ninguna prueba :( Sigue intentandolo."
                else:
                    comentario = "Dijkstra ha pasado alguna de las pruebas, revisa el codigo."

            print("Testeado "+str(dijkstra_method).split(" ")[1]+": "+str(aciertos)+"/"+str(NUM_PRUEBAS)+" pruebas superadas. "+comentario)
    except Exception as err:
        traceback.print_tb(err.__traceback__)
        traceback.print_exc()
        if str(graph_class) == NX_GRAPH:
            print("\n\n[ALERTA!!]: Estas usando la clase NetworkX como libreria de grafos. Algunas llamadas a esta libreria no son iguales que las que usamos para 'nuestra' libreria Grafos. Comprueba que tu Dijkstra se ejecuta sin error antes de probarlo con el tester.")
        else:
            print("\n\nAlgo ha fallado durante las pruebas. Consulta con tu querido profesor de practicas.")
