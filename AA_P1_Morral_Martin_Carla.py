
# coding: utf-8

# <div style="padding:30px; color: white; background-color: #0071CD">
# <center>
# <img src="img/logoub.jpeg"></img>
# <center>
# <p>
# <h1>Algorísmica Avançada</h1>
# <h2>Pràctica 1 - Grafs </h2>
# </center>
# </p>
# </div>

# <div class="alert alert-danger" style="width:95%; margin:0 auto; padding">
# <center><p><h2> ¡¡IMPORTANTE!! </h2></p> </center> 
# 
# <p>
# Para la realizacón de esta práctica tendréis que utilizar vuestra propia clase `Graph` implementada en la Práctica 0. Si dicha clase no cumple los requisitos funcionales y no pasa los tests, con muy alta probabilidad los tests propuestos para esta práctica tampoco funcionen correctamente. 
# </p>
# </div>

# <div class="alert alert-info">
# <center>
#   <h1>Introducción</h1>
# </center>

# 
# A lo largo de esta práctica trabajaremos con el grafo generado a partir de la red de metro de Londres. En este grafo los nodos representan las estaciones y los ejes las vias que van de una estación a otra. Todos los ejes tienen cuatro atributos:
# 
# * Linea
# * Color
# * Nombre (de la linea)
# * Distancia
# 

# In[1]:

import networkx as nx
from util import get_subway_graph, draw_subway_graph
G = nx.Graph()
G, lines = get_subway_graph('csv', nx.graph.Graph)
G.edge[1][52]


# Los nodos contienen el nombre de la estación, la latitud y longitud a la que está situada la estación, el número de lineas y la zona.

# In[2]:

G.node[124]


# Para mas consultas, la información ha sido extraida de Wikimedia Commons:
# 
# https://commons.wikimedia.org/wiki/London_Underground_geographic_maps/CSV

# # util.py
# 
# En este archivo se os facilitan tres funciones que os permitiran cargar y visualizar la red de metro.
# ```python
# """
# Retorna un objeto nx.Graph que corresponde al grafo de la red de metro y un 
# diccionario con las lineas del metro
#  - location: ruta donde esta almacenado el archivo .csv
#  - Klass: la clase Graph que hemos desarrollado en la práctica 0
# """
# G, lines = get_subway_graph(location, Klass)
# 
# """
# Dibuja el grafo que le pasemos por parametro.
# - G: Grafo de la red de metro
# - lines: diccionario con la información sobre las lineas del metro
# - figsize: parametro opcional que nos permite definir el tamaño de la figura
# - show_labels: parametro opcional que nos permite indicar si queremos mostrar los 
#     nombres de las estaciones
# """
# draw_metro_graph(G, lines, figsize=(10,6), show_labels=False)
# 
# ```

# In[4]:

draw_subway_graph(G, lines, figsize=(10,6))


# <div class="alert alert-info">
# <center>
#   <h1>Contenido</h1>
#   </center><p>
# 
# 

# <div class="alert alert-success" style="width:90%; margin:0 auto;">
# 
#   <h2><p>1- Dijkstra</p></h2>
#   
#   <p>
#  Se propone que hagáis dos implementaciones del algoritmo <a href="https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm">Dijkstra</a>. 
#  <ul><li>
#  **dijkstra1:** La primera consiste en el algoritmo común en el que tendremos en cuenta las distancias definidas en los edges. 
#  </li><li>
#  **dijkstra2:** La segunda implementación es simplemente una ampliación del primer algoritmo en el que añadiremos una penalización por cambio de linea.</li>
#  </ul>
# </p>
# <h3> Ejemplo _dijkstra2_</h3>
# <p>
# Para ilustrar lo que se pide en la práctica veamos como se calcularía el valor de **C** y **D** en el siguiente gráfico:
# </p>
# 
# 
# <p><center><img src='img/e1.png'></img></center></p>
# 
# 
# <p>
# <ul><li>
# **[C]** Calculamos V(B) como V(A) + d([A,B]), calculamos V(C) como V(B) + d([B,C]), ya hemos acabado.
# </li>
# <li>
# **[D]** Calculamos V(B) como V(A) + d([A,B]), calculamos V(D) como V(B) + d([B,D]), como [A,B] es linea azul y [B,D] es linea verde, añadimos una penalización por el transbordo a la que llamamos **P**. Por lo tanto, al final obtenemos que V(D) = V(B) + d([B,C]) + P
# </li>
# </ul>
# 
# <br>
# -**V(X)**: Valor de X
# <br>
# -**d([X,Y])**: Distancia de X a Y
# <br>
# -**P**: Penalización por transbordo
# 
# </p>
# 

# <div class="alert alert-danger" style="width:80%; margin:0 auto; padding">
# <center><p><h3> Código </h3></p> </center>
# <p>
# <h3>INPUT</h3>
# <ul>
# <li>__G__: Este es el grafo (en el caso de esta práctica la red de metro) que utilizaremos para buscar el camino. Debe de ser un objeto de tipo `Graph` como el que habéis implementado en la Práctica 0.</li>
# <li>__origen__: Este parámetro corresponde al índice de un nodo. En este caso, como indexamos los nodos con el identificador de las paradas de Metro, deberá ser un entero _(e.g. 231)_.</li>
# <li>__destino__: El indice del nodo al que queremos llegar.</li>
# <li>__infinity=*(int)*__: Parametro opcional en el que definimos que numero nos va bien para utilizar como infinito en el momento de inicializar los pesos de los nodos.</li>
# <li>__penalty=*(int)*__: (_Solo necesario para dijkstra2)_ Es un numero entero que corresponde a la penalización que aplicamos al valor de un nodo cuando hay cambio de linea.
# </ul>
# <br>
# <h3>OUTPUT</h3>
# El output de la función es un diccionario que contiene los siguientes valores:
# <ul>
# <li>__ _'path'_ __: Una lista de índices correspondientes al camino encontrado del nodo inicial al nodo final __ambos nodos, inicio y final, han de estar incluidos en esta lista__.</li>
# <li>__ _'expanded'_ __: El numero de nodos que se han visitado para encontrar la solución.</li>
# <li>__ _'distance'_ __: La distancia del camino mínimo desde el origen hasta el destino (es decir, el valor del nodo destino).
# <ul>
# 
# </p>
# </div>
# 

# In[11]:

import heapq
def dijkstra(G, origen, destino, infinity=float('inf')):
    
    """
    Encuentra el camino mínimo entre dos nodos.
    """
    
    #Creamos un set de nodos:
    Q = set(G.nodes())
    
    #Definimos distancia inicial de cada nodo a infinito, menos la del nodo origen, que es cero. 
    #El diccionario prev nos da información sobre el nodo que le ha sumado la ultima distancia a un nodo n.
    #Con este último podremos encontrar el camino
    #La cola H nos permitirá ir sacando el nodo con distancia mínima de manera muy eficiente.
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
    
    #Empezamos con el algoritmo
    while len(Q) != 0:
        
        #Sacamos el menor nodo de la cola
        parent = heapq.heappop(H)[1]
        
        #Si el nodo no ha sido expandido lo eliminamos del set Q.
        Q.discard(parent)

        #Incrementamos la variable expanded.
        expanded += 1

        #Condiones de salida del bucle:
        #1a condición: No podemos continuar porque el grafo es inconexo.
        if(dist[parent] == infinity): return "Grafo inconexo"

        #2a condición: Hemos terminado el algoritmo. Ahora hemos de reconstruir el camino.
        if(parent == destino): 
            return construct_path(G, prev, destino, expanded, origen, dist)

        #Miramos los vecinos del nodo padre
        for neightbor in G.neighbors(parent):

            #Si el vecino no ha sido expandido:
            if neightbor in Q:
                alt = dist[parent] + G.edge[parent][neightbor]['distance']

                #Condición para la asignación de una nueva distancia.
                if alt < dist[neightbor]:
                    dist[neightbor] = alt
                    prev[neightbor] = parent
                    heapq.heappush(H, (alt, neightbor))
                        

def construct_path(G, prev, destino, expanded, origen, dist):
    
    """
    Reconstruye el camino desde el nodo destino hasta el nodo origen.
    """
      
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


# In[12]:

dijkstra(G, 10, 235)


# In[15]:

def dijkstra2(G, origen, destino, penalty=5000000, infinity=float('inf')):
    
    """
    Generalización de Dijkstra, ya que en Dijksta penalty = 0.
    """
    
    #Creamos un diccionario que tiene como claves los nodos y como valores una lista de sus linias.
    linies = {node:[] for node in G.nodes()}
    for node in G.nodes():
        for neighbor in G.neighbors(node):
            if(G.edge[node][neighbor]['line'] not in linies[node]):
                linies[node].append(G.edge[node][neighbor]['line'])

    #Creamos un set Q de la forma (nodo,linea). Cada nodo tendra una tupla en Q por cada linea que tenga.
    Q = []
    for node in linies:
        for i in linies[node]:
            Q.append((node, i))
    Q = set(Q)
    
    #Inicializamos el minheap y los diccionarios de distancias y prev. También la variable expanded.
    H = [] 
    dist = {}
    prev = {}
    expanded = 0

    #La variable node, en este caso será de la forma (nodo,linea) a diferencia del dijkstra normal.
    #Nos permitirá tener tantos nodos x como lineas a las que pertenezca x.
    #Así pues, cada nodo vendrá unequívocamente determinado por la tupla de su valor y la línia a la que pertenece.
    for node in Q:
        if node[0] == origen:
            dist[node] = 0   
            heapq.heappush(H, (0, node))
        else:
            dist[node] = infinity
            heapq.heappush(H, (infinity, node))
        prev[node] = None
    
    #Ahora hemos de aplicar dijkstra normal con alguna pequeña modificación.
    while len(Q) != 0:

        #Sacamos de la cola el nodo con distancia mínima.
        parent = heapq.heappop(H)[1]
        
        #Si el parent no ha sido expandido lo eliminamos del set Q.
        Q.discard(parent)

        #Actualizamos expanded
        expanded += 1

        #Condiciones de salida del while, la primera porque no podemos continuar y la segunda porque hemos terminado.
        if(dist[parent] == infinity): return "Grafo inconexo"
        if(parent[0] == destino): 
            return construct_path3(prev, destino, expanded, origen, dist, parent[1])


        #Miramos los vecinos del padre.
        for neighbor in G.neighbors(parent[0]):

            #Solo miramos los vecinos de la misma linea.
            neighbor_node = (neighbor,parent[1])

            #Miramos solo los que no hayan sido expandidos.
            if neighbor_node in Q and G.edge[neighbor][parent[0]]['line'] == parent[1]:

                alt = dist[parent] + G.edge[parent[0]][neighbor_node[0]]['distance']
                #Condición para la asignación de una nueva distancia.
                if alt < dist[neighbor_node]:
                    dist[neighbor_node] = alt
                    prev[neighbor_node] = parent
                    heapq.heappush(H, (alt, neighbor_node))


        #Este bucle nos permite asignar las penalty.
        #Miramos las tuplas formadas por (padre,linia) tales que linia!=linia_padre.
        for i in range(0,len(linies[parent[0]])):

            #Cogemos la tupla.
            neighbor_node = (parent[0],linies[parent[0]][i])

            #Tenemos que comprovar que la tupla no sea el mismo padre (que no tengan la misma linea)
            if(neighbor_node[1] != parent[1]):
                if neighbor_node in Q:

                    #En este caso hay que sumar la penalty
                    alt = dist[parent] + penalty
                    if alt < dist[neighbor_node]:
                        dist[neighbor_node] = alt
                        prev[neighbor_node] = parent
                        heapq.heappush(H, (alt, neighbor_node))
                        

def construct_path3(prev, destino, expanded, origen, dist, linia):
    
    """
    Reconstruye el camino desde el nodo destino hasta el nodo origen.
    """

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




# In[16]:

dijkstra2(G, 10, 235)


# <div class="alert alert-warning" style="width:80%; margin:0 auto; padding">
# <center><p><h3> Comentarios Dijkstra</h3></p> </center> </div>

# ### _(En esta sección se os propone explicar como habeis realizado la implementación y cual es la complejidad detallada del algoritmo. Podéis contestar en este mismo bloque)_
# 
# ### Implementación:
# Primero de todo, hemos inicializado una serie de estructuras de datos:
# 
# a) Un diccionario que contiene como claves a los nodos y como valores infinito,pues de entrada es la distancia 
# que hay entre cada nodo y el nodo origen. Claramente, el valor de origen es 0.
# 
# b)Tambien hemos creado una cola prioritaria (minheap), que contiene tuplas cuyo primer valor son las distancias
# y el segundo valor el nodo. Nos servirá para ir haciendo "pop" de los nodos que tengan la menor distancia en cada caso.
# 
# c) Un set Q que tendrá a los nodos, que nos será muy util para comprovar si un nodo ha sido expandido. Iremos quitando del set nos nodos que expandan.
# 
# d)Un diccionario prev, que tendrá la información de los padres (del nodo que te ha actualizado la distancia) y nos permitirá reconstruir el camino.
# 
# Ahora, utilizamos un algorismo iterativo que realiza los siguientes pasos:
# 1. Eliminamos el primer elemento del minheap. 
# 2. Este será el nodo a expandir (parent) siempre y cuando no haya sido expandido (o sea, que pertenezca a Q). Un nodo puede salir de H y haber sido expandido porque puede ser que nodo haya sido vecino de otro y se le haya actualizado la distancia en H (habiéndolo metido otra vez en H y por tanto es susceptible de volver a ser escogido). Si cumple la condición, se elimina de Q y se prosigue.
# 3. El recorrido iterativo se realizará hasta que tengamos un nodo cuya distancia sea infinita (el grafo es inconexo y por tanto no podremos continuar), o bien que sea el nodo destino (ya habremos finalizado la búsqueda).
# 4. Visitamos cada vecino del nodo parent. Si la suma de la distancia del nodo expandido más la distancia entre ambos nodos es menor a la distancia del nodo vistado, actualizamos su distancia en el diccionario y lo añadimos a la cola con la distancia. También actualizamos su predecesor en el diccionario prev (será el padre).
# 5. Volvemos al paso 1.
# 
# Una vez hayamos finalizado la busqueda, tendremos que el nodo actual a expandir corresponde con el destino. De esta manera, podemos buscar el padre del destino en el diccionario e ir haciendo un recorrido hacia atras hasta llegar al origen. Así tendremos el camino. Como lo tenemos invertido (de destino a origen), lo invertimos y ya tenemos el camino de origen a destino. Esto es lo que hace la función construct_path.
# 
# ### Complejidad:
# Ponemos |V| la cantidad de nodos |E| la cantidad de aristas.
# 
# El coste por rellenar el diccionario de distancias es de |V|.
# 
# El coste por rellenar el diccionario prev es de |V|.
# 
# El coste por rellenar el set Q es de |V|.
# 
# El coste por insertar los |V| nodos en el minheap será de log(|V|-1) x log(|V|-2) x .... x (log1) x 1 = log(|V|(|V|-1)/2)), ya que el algoritmo expandirá (mirará sus vecinos) de como máximo |V|-1 nodos  y el coste por insertar un elemento en un heap es O(logn), n = numero de elementos ya en el heap. Por tanto, el coste de estas cuatro acciones será de O(3|V|+log(|V|(|V|-1)/2)) $\approx$ O(|V|+log(|V|^2)) $\approx$ O(|V|+log(|V|))
# 
# El coste de eliminar un nodo de Q es 1. Como el algoritmo expandirá como máximo |V|-1 nodos el coste será |V|-1 del total de las eliminaciones de Q. |V|-1 $\approx$ |V|.
# 
# El coste de añadir un nuevo elemento al minheap es de O(log|V|). En el peor de los casos, actualizaremos las distancias de todos sus vecinos excepto de su padre cada vez que visitemos un nodo, es decir, actualizaremos tantas veces como edges. El coste de insertar |E| veces será por tanto de O(|E|log(|V|)).
# 
# El coste de la función construct_path (para encontrar el camino una vez llegado al destino), como el coste de visitar en el diccionario es de 1 y el de añadir un elemento a una array es también de 1, será de O(2|V|) como máximo, ya que en el peor de los casos para cada nodo se irá mirando su padre sin dejarnos a ningún nodo y el camino justamente será pasando por todos los nodos.
# 
# Por tanto, la complejidad total será de O(2|V| + log(|V|) + |E|log(|V|)) $\approx$ O(|V|+|E|log(|V|))
# 
# ### La complejidad del algoritmo dijkstra es de O(|V|+|E|log(|V|)).

# <div class="alert alert-warning" style="width:80%; margin:0 auto; padding">
# <center><p><h3> Comentarios Dijkstra2</h3></p> </center> </div>

# ### _(En esta sección se os propone explicar como habeis realizado la implementación y cual es la complejidad detallada del algoritmo. Podéis contestar en este mismo bloque)_
# 
# ### Implementación:
# Este es un algoritmo que parte totalmente de la base de dijkstra normal, es mas, es una generalización. Así pues, hemos trabajado sobre la base de dijkstra haciendo algunos canvios sin tocar nada de nada del proceso de implementación de dijkstra normal para adaptarlo finalmente a dijkstra2.
# Primero de todo, hemos inicializado una serie de estructuras de datos pero esta vez un poco distintas, ya que lo que buscamos en esta implementación es "transformarla" a dijkstra normal: Por ello, hemos "etiquetado" a cada uno de los nodos del grafo para que sean 100% exclusivos, es decir, un nodo 'x' que pertenezca a 'y' líneas, no será un nodo, sino 'y' nodos de la forma (nodo,linea_1),(nodo,linea_2),...,(nodo,linea_y), ya que cuando se produce un cambio de linea estos nodos también han de ser contabilizados como vecinos entre ellos. Será en este instante cuando empiecen a dar juego las penalties. Así pues, empezamos definiendo nuestras nuevas estructuras:
# 
# a) Un diccionario lineas de la forma {nodo:[lineas]} donde [lineas] es una lista de sus lineas. Por ejemplo, si un nodo tiene las lineas 1, 5 y 8, la clave del diccionario será 'nodo' y el valor [1,5,8]. Este diccionario nos servirá para el primer bucle del paso 5.
# 
# b) Un diccionario de la forma {(nodo,linea):distancia}. La distancia será inicialmente 0 para las tuplas (origen,linea_1),...,(origen,linea_n) y para las demás será de infinito.
# 
# c)Tambien hemos creado una cola prioritaria (minheap), que contiene tuplas cuyo primer valor son las distancias
# y el segundo valor las tuplas ya mencionadas de la forma (nodo,linea). Nos servirá para ir haciendo "pop" de las tuplas que tengan la menor distancia en cada caso.
# 
# d) Un set Q que tendrá a todas las tuplas, que nos será muy util para comprovar si una tupla ha sido expandida. Iremos quitando del set las tuplas que expandan.
# 
# Ahora, utilizamos un algorismo iterativo que realiza los siguientes pasos:
# 1. Eliminamos el primer elemento del minheap. 
# 2. Este será el (nodo,linea) a expandir (parent) siempre y cuando no haya sido expandido (o sea, que pertenezca a Q). Una tupla puede salir de H y haber sido expandida porque puede ser que nodo haya sido vecino de otro y se le haya actualizado la distancia en H (habiéndolo metido otra vez en H y por tanto es susceptible de volver a ser escogido). Si cumple la condición, se elimina de Q y se prosigue.
# 3. El recorrido iterativo se realizará hasta que tengamos un (nodo,linea) cuya distancia sea infinita (el grafo es inconexo y por tanto no podremos continuar), o bien que la primera posición de la tupla sea el nodo destino (ya habremos finalizado la búsqueda).
# 4. Visitamos cada vecino del (padre,linea_padre) siempre y cuando al vecino lo visitemos de la forma (vecino,linea_padre) y este no haya sido padre, o sea, que aún pertenezca a Q. Si la suma de la distancia del (padre,linea_padre) expandido más la distancia entre ambos nodos es menor a la distancia del (vecino,linea_padre) vistado, actualizamos su distancia en el diccionario y lo añadimos a la cola con la distancia. También actualizamos su predecesor en el diccionario prev (será el entonces la tupla (padre,linea_padre)).
# 5. Como hemos dicho antes, ahora necesitamos ver también a los vecinos del (padre,linea_padre) de la forma (padre,linea_1),...,(padre,linea_n) excepto él mismo, claro está. Ahora explicamos porqué: como hemos dicho al principio, cada nodo viene únicamente determinado por su valor y la línea a la que pertenece. Por tanto, un nodo (nodo,linea_1) necesariamente ha de ser vecino de un nodo (nodo,linea_2) y así sucesivamente. Y esto lo tenemos que hacer a parte en un segundo bucle porque la función G.neighbors(nodo) no "sabe" que estamos "desdoblando", "tripleando", etc los nodos teniendo en cuenta sus líneas. **ESTE PASO NOS PERMITIRÁ "CAMBIAR DE ANDÉN" Y ASÍ PUES AÑADIREMOS LAS PENALTIES SOLAMENTE AL CAMINO CORRESPONDENTE AL CAMBIO DE ANDÉN**, que se producirá cuando visitemos a estos (padre,linea) tales que linea != linea_padre y su distancia en el diccionario sea mayor a la PENALTY + la distancia de (padre,linea_padre) en el diccionario. Será entonces cuando (si se cumple dicha condición) se actualice la distancia de (padre,linea) a la de (padre,linea_padre) + PENALTY y además, también actualizaremos el predecesor de (padre,linea) en el diccionario prev a (padre,linea_padre). Como en el bucle anterior, evidentemente antes de proceder a comprovar lo de las distacncias también se ha de cumplir que un (padre,linea) no haya sido padre o sea, que aún pertenezca a Q. 
# 6. Volvemos al paso 1.
# 
# La función construct_path, en este caso tiene una ligera modificación, ya que los elementos de prev no son nodos, son tuplas (nodo,linea). Lo único que tenemos en cuenta es esto para ir rehaciendo el camino y además también tenemos que controlar que un nodo no aparezca más de una vez seguida en el array resultado, ya que puede producirse si se hace un cambio de linea.
# 
# ### Complejidad:
# Ponemos |V| la cantidad de nodos, |E| la cantidad de aristas y n_i la cantidad de lineas de cada nodo.
# 
# El coste por rellenar el diccionario lineas es de |V|(n_1+n_2+...+n|V|) $\approx$ (n_1+n_2+...+n|V|)
# 
# El coste por rellenar el diccionario de distancias es de |V|(n_1+n_2+...+n|V|) $\approx$ (n_1+n_2+...+n|V|)
# 
# El coste por rellenar el diccionario prev es de |V|(n_1+n_2+...+n|V|) $\approx$ (n_1+n_2+...+n|V|)
# 
# El coste por rellenar el set Q es de |V|(n_1+n_2+...+n|V|) $\approx$ (n_1+n_2+...+n|V|)
# 
# Denotaremos |n| = (n_1+n_2+...+n|V|), |En| = numero de aristas "nuevas" + |E|
# 
# El coste por insertar los |n| nodos en el minheap será de log(|n|-1) x log(|n|-2) x .... x (log1) x 1 = log(|n|(|n|-1)/2)), ya que el coste por insertar un elemento en un heap es O(logn), n = numero de elementos ya en el heap. Por tanto, el coste de estas cuatro acciones será de O(3|n|+log(|n|(|n|-1)/2)) $\approx$ O(|n|+log(|n|^2))
# 
# El coste de eliminar un nodo de Q es 1. Como el algoritmo expandirá como máximo |n|-1 nodos el coste será |n|-1 del total de las eliminaciones de Q. |n|-1 $\approx$ |n|.
# 
# El coste de añadir un nuevo elemento al minheap es de O(log|V|). En el peor de los casos, actualizaremos las distancias de todos sus vecinos excepto de su padre cada vez que visitemos un nodo, es decir, actualizaremos tantas veces como edges |En|. El coste de insertar |En| veces será por tanto de O(|En|log(|n|)).
# 
# El coste de la función construct_path (para encontrar el camino una vez llegado al destino), como el coste de visitar en el diccionario es de 1 y el de añadir un elemento a una array es también de 1, será de O(2|n|) como máximo, ya que en el peor de los casos para cada nodo se irá mirando su padre sin dejarnos a ningún nodo y el camino justamente será pasando por todos los nodos.
# 
# Por tanto, la complejidad total será de O(2|n| + log(|n|) + |E|log(|n|)) $\approx$ O(|n|+|En|log(|n|))
# 
# ### La complejidad del algoritmo dijkstra es de O(|n|+|En|log(|n|)).

# <div class="alert alert-success" style="width:90%; margin:0 auto;">
# 
#   <h2><p>2- Matriz de distancias</p></h2>
#    <p>
#  Se propone el desarrollo de un algoritmo que dado un grafo construya una matriz de $V \times V$ (donde $V$ es el número de vértices del grafo) que contenga la distancia mínima de cada vértice al resto. 
# </p>
# 
# <p></p>
# 
# <p>
# <b> Nota: </b> Recordad que en Algorítmica Avanzada buscamos la implementación de algoritmos que no solo resuelvan el problema, sino que lo hagan de manera eficiente
# </p>
# 

# <div class="alert alert-danger" style="width:80%; margin:0 auto; padding">
# <center><p><h3> Código </h3></p> </center>
# <p>
# <h3>INPUT</h3>
# <ul>
# <li>__G__: Este es el grafo (en el caso de esta práctica la red de metro) sobre el que calcularemos la matriz de distancias. Debe de ser un objeto de tipo `Graph` como el que habéis implementado en la Práctica 0.</li>
# </ul>
# <br>
# <h3>OUTPUT</h3>
# <ul>
# <li>El output de la función es un diccionario cuya clave es un nodo (origen) y el valor es otro diccionario con la forma $\{nodo\_destino: distancia\}$.</li>
#     </ul>
# 
# </p>
# </div>
# 

# In[21]:

def dist_mat(G):
    
    """
    Calcula todas las distancias entre todos los nodos ejectutando Dijkstra n veces.
    """
    
    #Creamos el diccionario result, que tendrá la información del output
    result = {}
    
    #Ejecutaremos Dijkstra3 n veces, una para cada nodo
    for node in G.nodes():
        result[node] = dijkstra3(G,node)
    
    return result
    
    
def dijkstra3(G, origen, infinity=float('inf')):
    
    """
    Dijkstra sin destino. Función que devuelve un diccionario que contiene como claves a los nodos y como valores 
    a las distancias de cada nodo al nodo inicial. No hace falta reconstruir el camino.
    """
    
    #Inicializamos las listas y los diccionarios.
    #En este caso no hace falta el diccionario prev ya que no devolvemos el path.
    Q = set(G.nodes())
    H = [] 
    dist = {}
    expanded = 0

    #Definimos distancia inicial de cada nodo a infinito, menos la del nodo origen, que es cero. 
    #La cola H nos permitirá ir sacando el nodo con distancia mínima de manera muy eficiente.
    for node in G.nodes():
        if node == origen:
            dist[node] = 0   
            heapq.heappush(H, (0, node))
        else:
            dist[node] = infinity
            heapq.heappush(H, (infinity, node))
    
    #Empezamos con el algoritmo.
    #En este caso la única condición de salida del bucle es que hayamos acabado de mirar todos los elementos del set.
    while len(Q) != 0:
        
        #Sacamos el elemento mínimo de la cola.
        parent = heapq.heappop(H)[1]
        
        #Si este elemento no ha sido expandido lo eliminamos del set Q.
        Q.discard(parent)

        #Incrementamos expanded.
        expanded += 1

        #Miramos los vecinos del nodo padre.
        for neightbor in G.neighbors(parent):
            #Si ese vecino no ha sido expandido.
            if neightbor in Q:
                alt = dist[parent] + G.edge[parent][neightbor]['distance']

                #Condición para la asignación de una nueva distancia.
                if alt < dist[neightbor]:
                    dist[neightbor] = alt
                    heapq.heappush(H, (alt, neightbor))
    
    #Devolvemos un diccionario de la forma {nodo: distancia}, donde distancia es la distancia entre nodo y origen.
    return dist


# In[22]:

dist_mat(G)


# <div class="alert alert-warning" style="width:80%; margin:0 auto; padding">
# <center><p><h3> Comentarios Matriz de Distancias</h3></p> </center> </div>

# ### _(En esta sección se os propone explicar como habeis realizado la implementación y cual es la complejidad detallada del algoritmo. Podéis contestar en este mismo bloque)_
# 
# ### _(En esta sección se os propone explicar como habeis realizado la implementación y cual es la complejidad detallada del algoritmo. Podéis contestar en este mismo bloque)_
# 
# ### Implementación:
# Primero de todo, hemos inicializado una serie de estructuras de datos:
# 
# a) Un diccionario result, que contendrá el resultado.
# 
# Dentro de dijkstra3:
# 
# a) Un diccionario que contiene como claves a los nodos y como valores infinito,pues de entrada es la distancia 
# que hay entre cada nodo y el nodo origen. Claramente, el valor de origen es 0.
# 
# b)Tambien hemos creado una cola prioritaria (minheap), que contiene tuplas cuyo primer valor son las distancias
# y el segundo valor el nodo. Nos servirá para ir haciendo "pop" de los nodos que tengan la menor distancia en cada caso.
# 
# c) Un set Q que tendrá a los nodos, que nos será muy util para comprovar si un nodo ha sido expandido. Iremos quitando del set nos nodos que expandan.
# 
# Ahora, utilizamos un algorismo iterativo que realiza los siguientes pasos:
# 0. Para cada nodo, se llama a dijkstra3 con el nodo como origen. Dik¡jkstra3 devuelve un diccionario de la forma 
# 1. El recorrido iterativo se realizará hasta que la longitud de Q sea 0, es decir, hasta que todos los nodos hayan sido expandidos, pues en este caso solo buscamos las distancias, no un camino.
# 2. Eliminamos el primer elemento del minheap. 
# 3. Este será el nodo a expandir (parent) siempre y cuando no haya sido expandido (o sea, que pertenezca a Q). Un nodo puede salir de H y haber sido expandido porque puede ser que nodo haya sido vecino de otro y se le haya actualizado la distancia en H (habiéndolo metido otra vez en H y por tanto es susceptible de volver a ser escogido). Si cumple la condición, se elimina de Q y se prosigue.
# 4. Visitamos cada vecino del nodo parent. Si la suma de la distancia del nodo expandido más la distancia entre ambos nodos es menor a la distancia del nodo vistado, actualizamos su distancia en el diccionario y lo añadimos a la cola con la distancia.
# 5. Volvemos al paso 1.
# 
# Una vez hayamos finalizado el bucle, solamente se devuelve el diccionario de las distancias, que ya contiene toda la información que nos interesa.
# 
# ### Complejidad:
# Ponemos |V| la cantidad de nodos |E| la cantidad de aristas.
# 
# El coste por rellenar el diccionario result es de |V|.
# 
# El coste por rellenar el diccionario de distancias es de |V|.
# 
# El coste por rellenar el set Q es de |V|.
# 
# El coste por insertar los |V| nodos en el minheap será de log(|V|-1) x log(|V|-2) x .... x (log1) x 1 = log(|V|(|V|-1)/2)), ya que el algoritmo expandirá (mirará sus vecinos) de como máximo |V|-1 nodos  y el coste por insertar un elemento en un heap es O(logn), n = numero de elementos ya en el heap. Por tanto, el coste de estas cuatro acciones será de O(3|V|+log(|V|(|V|-1)/2)) $\approx$ O(|V|+log(|V|^2)) $\approx$ O(|V|+log(|V|))
# 
# El coste de eliminar un nodo de Q es 1. Como el algoritmo expandirá como máximo |V|-1 nodos el coste será |V|-1 del total de las eliminaciones de Q. |V|-1 $\approx$ |V|.
# 
# El coste de añadir un nuevo elemento al minheap es de O(log|V|). En el peor de los casos, actualizaremos las distancias de todos sus vecinos excepto de su padre cada vez que visitemos un nodo, es decir, actualizaremos tantas veces como edges. El coste de insertar |E| veces será por tanto de O(|E|log(|V|)).
# 
# Por tanto, la complejidad total será de O(log(|V|) + |E|log(|V|)) $\approx$ O(|E|log(|V|))
# 
# ### La complejidad del algoritmo es de O(|E|log(|V|)).
# 
# 

# <div class="alert alert-info">
# <center>
#   <h1>Entrega</h1>
# </center>
# <p>
# La entrega de esta práctica se podrá realizar en el campus virtual hasta el día __21 de Octubre a las 23:55__. En la tarea que se habilitará en el campus deberéis colgar __el archivo .ipynb__ asi como __el fichero .pyc con vuestra clase grafo__ con los nombres:
# </p>
# <p>
# ```
# * AA_P1_<apellido1>_<apellido2>_<nombre>.ipynb
# * graphs.pyc
# ```
# 
# </p>
# <p>
# Por ejemplo:
# </p>
# <p>
# ```
# AA_P1_Doe_Ritchie_John.ipynb
# ```
# 
# </p>
# <p>
# 
# Es fundamental que el código esté bién comentado y con un análisis de complejidad exhaustivo del algoritmo.
# </p>
# </div>
