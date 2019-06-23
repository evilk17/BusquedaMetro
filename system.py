#-*- coding: utf-8 -*-
"""
This graph solving system was created by Nicky García Fierros
for PROTECO's Inteligencia Artificial 2017-2 course.

Licensing Information:  You are free to use or extend these projects for
educational purposes provided that (1) you do not distribute or publish
solutions, (2) you retain this notice, and (3) you provide clear
attribution to Nicky García Fierros, including a link to
https://github.com/kotoromo.

"""

import util, solutions

class Problem:

    graph_2 = {
        'A':[('B', 2)],
        'B':[('C', 1), ('D', 5)],
        'D':[('E', 1), ('F', 3)],
        'E':[('D', 5)],
        'F':[('D', 5)]
    }

    """
    Grafo con costos
    """
    graph_1 = {
        'A':[('B', 1)],
        'B':[('C', 1), ('D', 1), ('A', 1)],
        'C':[('B', 1)],
        'D':[('E', 1), ('F', 1), ('B', 1)],
        'E':[('D', 1)],
        'F':[('D', 1)]
    }

    graph_3 = {
        'A':[('B', 3), ('C', 1)],
        'B':[('C', 1), ('D', 2), ('A', 3)],
        'C':[('B', 1), ('A', 1), ('F', 7)],
        'D':[('E', 2), ('F', 1), ('B', 2)],
        'E':[('D', 2)],
        'F':[('D', 1), ('C', 7)]
    }
    
    grafo = {
        'OCEANIA':[('PANTITLAN',3),('CONSULADO',3),('SAN_LAZARO',3)],
        'PANTITLAN': [('OCEANIA',3),('SAN_LAZARO',6),('JAMAICA',5)],
        'CONSULADO':[('OCEANIA',3),('MARTIN_CARRERA',3),('MORELOS',2),('LA_RAZA',3)],
        'SAN_LAZARO':[('PANTITLAN',6),('OCEANIA',3),('MORELOS',2),('CANDELARIA',2)],
        'JAMAICA':[('PANTITLAN',5),('SANTA_ANITA',1),('CHABACANO',1),('CANDELARIA',2)],
        'MARTIN_CARRERA':[('CONSULADO',3),('DEPORTIVO18',3)],
        'MORELOS':[('CONSULADO',3),('SAN_LAZARO',2),('CANDELARIA',1),('GARIBALDI',3)],
        'CHABACANO':[('JAMAICA',1),('SANTA_ANITA',2),('PINO_SUAREZ',2),('SALTO_AGUA',3),('CENTRO_MEDICO',2),('ERMITA',6)],
        'CANDELARIA':[('SAN_LAZARO',1),('MORELOS',1),('PINO_SUAREZ',2),('JAMAICA',2)],
        'SANTA_ANITA':[('JAMAICA',1),('CHABACANO',2),('ATLALILCO',6)],
        'PINO_SUAREZ':[('CANDELARIA',2),('CHABACANO',2),('SALTO_AGUA',2),('BELLAS_ARTES',3)],
        'DEPORTIVO18':[('MARTIN_CARRERA',2),('LA_RAZA',2),('INSTITUTO_PETROLEO',2)],
        'LA_RAZA':[('DEPORTIVO18',2),('CONSULADO',3),('GUERRERO',2),('INSTITUTO_PETROLEO',2)],
        'GARIBALDI':[('MORELOS',3),('GUERRERO',1),('BELLAS_ARTES',1)],
		'GUERRERO':[('GARIBALDI',1),('LA_RAZA',2),('HIDALGO',1)],
		'HIDALGO':[('GUERRERO',1),('BELLAS_ARTES',1),('BALDERAS',2),('TACUBA',7)],
        'INSTITUTO_PETROLEO':[('LA_RAZA',2),('DEPORTIVO18',2),('ROSARIO',6)],
        'BELLAS_ARTES':[('HIDALGO',1),('PINO_SUAREZ',3),('GARIBALDI',1),('SALTO_AGUA',2)],
        'SALTO_AGUA':[('BALDERAS',1),('PINO_SUAREZ',2),('BELLAS_ARTES',2),('CHABACANO',3)],
        'BALDERAS':[('HIDALGO',2),('SALTO_AGUA',1),('CENTRO_MEDICO',3),('TACUBAYA',6)],
        'CENTRO_MEDICO':[('BALDERAS',3),('CHABACANO',2),('TACUBAYA',3),('ZAPATA',4)],
        'ATLALILCO':[('SANTA_ANITA',6),('ERMITA',2)],
        'ZAPATA':[('ERMITA',3),('CENTRO_MEDICO',4),('MIXCOAC',3)],
        'ERMITA':[('ZAPATA',3),('ATLALILCO',2),('CHABACANO',6)],
        'MIXCOAC':[('ZAPATA',3),('TACUBAYA',3)],
        'TACUBAYA':[('TACUBA',5),('CENTRO_MEDICO',3),('BALDERAS',6),('MIXCOAC',3)],
        'TACUBA':[('TACUBAYA',5),('ROSARIO',4),('HIDALGO',7)],
        'ROSARIO':[('INSTITUTO_PETROLEO',6),('TACUBA',4)]




		
        
    }



    """Creo una lista indicando a que lineas pertenece cada estacion para realizar la heuristica
    La heuristica es que si el nodo actual pertenece ala misma linea le dara preferencia que a otra estacion que 
    pertenesca a otra linea. La Linea B sera indicada con el nùmero 11"""
    lineaEstaciones={
        'OCEANIA':[5,11],
        'PANTITLAN': [1,5,9],
        'CONSULADO':[5,4],
        'SAN_LAZARO':[11,1],
        'JAMAICA':[4,9],
        'MARTIN_CARRERA':[4,6],
        'MORELOS':[4,11],
        'CHABACANO':[2,9,8],
        'CANDELARIA':[1,4],
        'SANTA_ANITA':[4,8],
        'PINO_SUAREZ':[2,1],
        'DEPORTIVO18':[3,6],
        'LA_RAZA':[5,3],
        'GARIBALDI':[11,8],
        'GUERRERO':[3,11],
        'HIDALGO': [2,3],
        'INSTITUTO_PETROLEO':[5,6],
        'BELLAS_ARTES':[2,8],
        'SALTO_AGUA':[1,8],
        'BALDERAS':[1,3],
        'CENTRO_MEDICO':[3,9],
        'ATLALILCO':[8,12],
        'ZAPATA':[3,12],
        'ERMITA':[2,12],
        'MIXCOAC':[7,12],
        'TACUBAYA':[1,9,7],
        'TACUBA':[2,7],
        'ROSARIO':[6,7]
    }

    """
    Problem abstraction.
    Arguments:
        'space' = state space [OPTIONAL]
        'goal' = goal state
        'start' = start state
        'goal_fn' = custom goal function, if not specified
         uses one that checks for equality between the state given
         and the goal state defined within the problem. [OPTIONAL]
        'heur' = heuristic function to use. [OPTIONAL]
        'suc_fn' = succesor function to use. [OPTIONAL]

    """
    

    def __init__(self, *args, **kwargs):
        # Dictionary which states the available default graphs.
        spaces = {'g1': self.graph_1, 'g2': self.graph_2, 'g3': self.graph_3, 'g':self.grafo}

        if kwargs is not None:
            if kwargs.get('space') is not None:
                if kwargs.get('space') in spaces.keys():
                    self.state_space = spaces.get(kwargs.get('space'))
                else:
                    self.state_space = kwargs.get('space')
            else:
                self.state_space = spaces['g1']

            self.goal_state = kwargs.get('goal')
            self.start_state = kwargs.get('start')

            if(kwargs.get('goal_fn') is None):
                self.goal_function = self.defaultIsGoal
            else:
                self.goal_function = kwargs.get('goal_fn')

            if kwargs.get('heur') is None:
                self.heuristic = self.nullHeuristic
            else:
                self.heuristic = kwargs.get('heur')

            if kwargs.get('suc_fn') is None:
                self.suc_fn = self.defautlSuccessorFunction
            else:
                self.suc_fn = kwargs.get('suc_fn')




        self.nodes_expanded = 0


    def mi_heuristica(self,nodoActual):
        #
        #
        for nodo in self.lineaEstaciones[nodoActual]:
            if nodo in self.lineaEstaciones[self.goal_state]:
                return 0
        return 1

    def getSuccessors(self, state):
        return self.suc_fn(state)

    """
    Successor function.
    Given a node, returns the list of successors associated with it.
    """
    def defautlSuccessorFunction(self, state):
        self.nodes_expanded += 1
        return self.state_space[state]

    """
    Trivial heuristic function
    """
    def nullHeuristic(self, *args, **kwargs):
        pass

    """
    Sets the heuristic function
    """
    def setHeuristic(self, h):
        self.heuristic = h

    """
    Default goal check function.
    Given a node, returns whether the node given is the same as the goal state.
    """
    def defaultIsGoal(self, state):
        return self.goal_state == state

    """
    Custom goal check function.
    Returns true if the given state is a goal.
    """
    def isGoal(self, state):
        return self.goal_function(state)

    """
    Start state getter.
    Returns the start state as defined by the problem.
    """
    def getStartState(self):
        return self.start_state

    """
    System is probably badly designed. This is the soulution I came up with.
    Method which finds the appropiate key for the given state.
    """
    def findKey(self, node_letter):
        keys = self.state_space.keys()
        for tuple in keys:
            if node_letter == tuple[0]:
                return tuple

    def restartCounter(self):
        self.nodes_expanded = 0

    def getNodesExpanded(self):
        a = self.nodes_expanded
        self.restartCounter()
        return a

    def getSolutionCost(self, solution):
        cost = 0
        if solution is None:
            return 0

        for i in range(0, len(solution)-1):
            #('A', 0):[('B', 3), ('C', 1)]
            # state_space = {(NODE, COST): [LIST OF TUPLES]}
            current = solution[i]
            successors = self.getSuccessors(current)

            #finding tuple with value
            for tuple in successors:
                if solution[i+1] == tuple[0]:
                    cost+=tuple[1]

        return cost



class Solver:
    """
    dfs ha sido implementado en tu lugar.
    """
    def dfs(self, problem):
        problem.restartCounter() #NO BORRAR!
        #Escribe tu código aquí

        fringe = util.Stack()
        visited = []
        plan = [problem.getStartState()]

        fringe.push( (problem.getStartState(), plan) )
        while(not fringe.isEmpty()):
            current, planToNode = fringe.pop()
            if problem.isGoal(current):
                #print("Visited: " + str(visited)) #DEBUG
                return planToNode
            if current not in visited:
                visited.append(current)
                for child in problem.getSuccessors(current):
                    fringe.push( (child[0], planToNode + [child[0]] ) )

    """
    Función que implementa Búsqueda por Amplitud (Breadth First Search)
    """
    def bfs(self, problem):
        problem.restartCounter() #NO BORRAR!
        #Escribe tu código aquí
		
        cola = util.Queue()
        visitados = []
        plan = [problem.getStartState()]
        cola.push( (problem.getStartState(), plan) )
        while(not cola.isEmpty()):
            current, planToNode = cola.pop()
            if problem.isGoal(current):
                #print("Visited: " + str(visited)) #DEBUG
                return planToNode
            if current not in visitados:
                visitados.append(current)
                for child in problem.getSuccessors(current):
                    cola.push( (child[0], planToNode + [child[0]] ) )

        #return solutions.Algorithms().bfs(problem)
        #util.raiseNotDefined()

    """
    Función que implementa Búsqueda de Coste Uniforme
    (conocido como Dijkstra o Uniform Cost Search)
    """
    def ucs(self, problem):
        problem.restartCounter() #NO BORRAR!
        costoDeRuta = 0

        rutaAlNodo = [problem.getStartState()]

        nodo_actual = (problem.getStartState(), rutaAlNodo, costoDeRuta)

        cola = util.PriorityQueue()

        cola.push(nodo_actual, 0)
        visitados = []

        while not cola.isEmpty():
        
            nodo_actual = cola.pop()

            if problem.isGoal(nodo_actual[0]):
                return nodo_actual[1]

            if  nodo_actual[0] not in visitados:

                visitados.append(nodo_actual[0])

                for hijo in problem.getSuccessors(nodo_actual[0]):
                    
                    hijo = (hijo[0], nodo_actual[1]+[hijo[0]], hijo[1]+nodo_actual[2])
                    
                    if hijo[0] not in visitados:
                        cola.update(hijo, hijo[2])
        
		#return solutions.Algorithms().ucs(problem)
        #util.raiseNotDefined()
    """
    Función que implementa A* (A estrella).
    """
    def astar(self, problem):
        problem.restartCounter()
        #Escribe tu código aquí
        problem.restartCounter() #NO BORRAR!
        costoDeRuta = 0

        rutaAlNodo = [problem.getStartState()]

        nodo_actual = (problem.getStartState(), rutaAlNodo, costoDeRuta)

        cola = util.PriorityQueue()

        print(nodo_actual[0])
        print(problem.goal_state)
        cola.push(nodo_actual, (0+problem.mi_heuristica(nodo_actual[0])))
        visitados = []

        while not cola.isEmpty():
        
            nodo_actual = cola.pop()

            if problem.isGoal(nodo_actual[0]):
                return nodo_actual[1]

            if  nodo_actual[0] not in visitados:

                visitados.append(nodo_actual[0])

                for hijo in problem.getSuccessors(nodo_actual[0]):
                    
                    hijo = (hijo[0], nodo_actual[1]+[hijo[0]], hijo[1]+nodo_actual[2]+problem.mi_heuristica(hijo[0]))
                    
                    if hijo[0] not in visitados:
                        cola.update(hijo, hijo[2])


        #return solutions.Algorithms().ucs(problem)
        #util.raiseNotDefined()

def main():
    """
    Objetivo:
    1. Crear un modelo del problema así como su planteamiento.

    Recuerda que un problema de búsqueda está definido por los siguientes
    elementos:
        a. Modelo del Mundo
        b. Estado Inicial/Estado Meta
        c. Función de sucesores
        d. Prueba de meta

    2. Implementar todos los algoritmos de búsqueda vistos en clase.
    Encontrar la solución del modelo del mundo utilizando cada una de las
    funciones implementadas (bfs, ucs y a*); DFS ha sido implementado para ti.

    Además, presentar los recorridos realizados por sus algoritmos paso a paso
    en hojas aparte para demostrar la correcta implementación de estos.

    Para ejecutar un problema y probar su solución es necesario hacer lo siguiente:

    # Crear una instancia de la clase 'Problem'. Verificar la definición
    del método constructor para los parámetros que puede tomar.

    # Definimos un problema con inicio en el nodo 'F' y meta en 'C' utilizando
    # El espacio de estados G3 (O el grafo 3)

    problem = Problem(goal='C', start = 'F', space = 'g3')

    # Después, debes crear una instancia de la clase 'Solver'.

    solver = Solver()

    # Debes imprimir la salida de los métodos 'bfs', 'ucs' y 'astar'
    # tal y como se muestra en el método principal main.

    print("Solución: " + str(solver.dfs(problem)))
    print("Nodos expandidos: " + str(problem.getNodesExpanded()))

    # Para pasar un grafo declarado en cualquier otra parte, puedes hacer uso del
    # parámetro 'space' del método constructor de la clase problem,
    # el cual es un diccionario como los demás grafos.

    # ej.
    problema = Problem(start = 'X', goal = 'Y', space = mi_grafo)
    solver = Solver()
    solucion = solver.dfs(problema)
    print "Resultado: %s"%(str(solucion))
    print "Nodos expandidos: %i"%(problem.getNodesExpanded())
    print "Costo: %i"%(problem.getSolutionCost(solucion))

    #Para implementar A*, necesitas desarrollar una heuristica admisible
    ( f(s) = g(s) + h(s) admisible <->  0 < h(s)<= g(s) )
    # Una vez hayas escrito tu heuristica, definela en el método constructor
    # del problema mediante el parámetro 'heur'
    # ej.
        def mi_heuristica(estado):
            ...

        problema = Problem(start = 'A', goal = 'F', space = 'g3', heur='mi_heuristica')
        solucionador = Solver(problema)
        solucionador.astar()
            ...


    Éxito y que te diviertas :)
    """

    """
    Programa en esta función tu heuristica
    """
    def heuristica(nodo):
        #Escribe tu código aquí

        #BORRA ESTA LINEA CUANDO HAYAS PROGRAMADO TU HEURISTICA
        return solutions.Algorithms().dist_heur(nodo)
    
    #Se define el problema inicial 
    problem = Problem(start = 'MARTIN_CARRERA', goal='ZAPATA', space='g', heur=heuristica)
    solv = Solver()
    #Probando heuristica Si las dos estaciones (Estado actual , Estado final) se encuentran en la misma linea
	#print(mi_heuristica('SAN_LAZARO','OCEANIA'));
    """
    Soluciones muestra para tu apoyo en el uso del sistema.
    Si te estorban puedes comentarlas sin problema o, en su defecto, borrarlas.
    """

    print("--------------DFS----------------------")
    sol = solv.dfs(problem)
    print "Solucion: %s" % (str(sol))
    print "Nodos expandidos: %s" % (str(problem.getNodesExpanded()))
    print "Costo: %i" % (problem.getSolutionCost(sol))

    print("--------------BFS----------------------")
    sol = solv.bfs(problem)
    print "Solucion: %s" % (str(sol))
    print "Nodos expandidos: %s" % (str(problem.getNodesExpanded()))
    print "Costo: %i" % (problem.getSolutionCost(sol))


    print("--------------UCS----------------------")
    sol = solv.ucs(problem)
    print "Solucion: %s" % (str(sol))
    print "Nodos expandidos: %s" % (str(problem.getNodesExpanded()))
    print "Costo: %i" % (problem.getSolutionCost(sol))

    print("--------------A*----------------------")
    sol = solv.astar(problem)
    print "Solucion: %s" % (str(sol))
    print "Nodos expandidos: %s" % (str(problem.getNodesExpanded()))
    print "Costo: %i" % (problem.getSolutionCost(sol))

    

    """
    Programa abajo tus soluciones o sustituye las de arriba por las tuyas.
    """




if __name__ == '__main__':
    main()
