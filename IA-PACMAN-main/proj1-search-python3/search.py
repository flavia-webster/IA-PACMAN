# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    from util import Stack  # Importa a classe Stack do arquivo util.py

    stack = Stack()  # Inicializa a pilha para armazenar os estados a serem explorados
    visited = set()  # Conjunto para armazenar os estados já visitados e evitar ciclos

    # Empilha o estado inicial e o caminho vazio (como ainda não se moveu, o caminho está vazio)
    stack.push((problem.getStartState(), []))

    while not stack.isEmpty():  # Enquanto houver estados na pilha
        state, path = stack.pop()  # Desempilha o estado atual e o caminho até ele

        if problem.isGoalState(state):  # Se o estado atual é o estado objetivo
            return path  # Retorna o caminho até o estado objetivo

        if state not in visited:  # Se o estado atual não foi visitado
            visited.add(state)  # Marca como visitado

            # Obtém os sucessores do estado atual e os empilha
            for nextState, action, cost in problem.getSuccessors(state):
                if nextState not in visited:  # Se o próximo estado não foi visitado
                    # Empilha o próximo estado e o caminho atualizado
                    stack.push((nextState, path + [action]))

    return []  # Se não encontrar o estado objetivo, retorna lista vazia


def breadthFirstSearch(problem):
    from util import Queue
    """Busca em largura no problema do Pacman."""
    # Cria uma fila para armazenar os estados a serem explorados, iniciando com o estado inicial
    # Cada item na fila é uma tupla (estado, caminho até agora)
    frontier = Queue()
    frontier.push((problem.getStartState(), []))

    # Cria um conjunto para armazenar os estados já visitados
    explored = set()

    # Enquanto ainda há estados na fronteira
    while not frontier.isEmpty():
        # Pega o próximo estado da fronteira
        state, actions = frontier.pop()

        # Se o estado é o objetivo, retorna as ações para chegar até aqui
        if problem.isGoalState(state):
            return actions

        # Se o estado não foi explorado ainda
        if state not in explored:
            # Marca como explorado
            explored.add(state)

            # Adiciona os sucessores à fronteira
            for successor, action, _ in problem.getSuccessors(state):
                if successor not in explored:
                    # Adiciona na fronteira com as ações atualizadas
                    frontier.push((successor, actions + [action]))

    # Retorna uma lista vazia se não encontrar um caminho
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    from util import PriorityQueue

    # A fila de prioridades, onde cada item é uma tupla (estado, caminho, custo), com o custo como prioridade
    frontier = PriorityQueue()
    # Adiciona o estado inicial à fronteira com um caminho vazio e custo zero
    frontier.push((problem.getStartState(), [], 0), 0)

    # Conjunto para armazenar os estados já visitados
    explored = set()

    while not frontier.isEmpty():
        # Pega o próximo estado da fronteira (com o menor custo total de caminho)
        state, path, current_cost = frontier.pop()

        # Se o estado é o objetivo, retorna o caminho até aqui
        if problem.isGoalState(state):
            return path

        # Se o estado não foi explorado ainda
        if state not in explored:
            # Marca como explorado
            explored.add(state)

            # Para cada sucessor do estado atual
            for successor, action, step_cost in problem.getSuccessors(state):
                if successor not in explored:
                    # Calcula o novo custo como sendo o custo atual mais o custo para chegar ao sucessor
                    new_cost = current_cost + step_cost
                    # Adiciona o sucessor à fronteira com o novo custo como prioridade
                    frontier.push((successor, path + [action], new_cost), new_cost)

    # Retorna uma lista vazia se não encontrar um caminho
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from util import PriorityQueue

    # Inicializa a fronteira usando uma fila de prioridades com o estado inicial
    frontier = PriorityQueue()
    startState = problem.getStartState()
    frontier.push((startState, [], 0), 0 + heuristic(startState, problem))

    # Inicializa o conjunto de estados explorados
    explored = set()

    while not frontier.isEmpty():
        # Pega o estado na fronteira com o menor custo estimado até o objetivo (custo até agora + heurística)
        currentState, actions, currentCost = frontier.pop()

        # Se o estado atual é o objetivo, retorna as ações para chegar até aqui
        if problem.isGoalState(currentState):
            return actions

        # Marca o estado atual como explorado
        if currentState not in explored:
            explored.add(currentState)

            # Itera sobre os sucessores do estado atual
            for nextState, action, cost in problem.getSuccessors(currentState):
                # Se o sucessor não foi explorado ou não está na fronteira
                if nextState not in explored:
                    # Calcula o novo custo como sendo o custo até o estado atual mais o custo para chegar ao sucessor
                    newCost = currentCost + cost
                    # Adiciona o sucessor à fronteira com a prioridade sendo o novo custo mais a heurística
                    frontier.update((nextState, actions + [action], newCost), newCost + heuristic(nextState, problem))

    # Retorna uma lista vazia se não encontrar um caminho
    return []



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
