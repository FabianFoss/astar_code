import Map

class Node(): #Instantiate nodes to search throu 

    def __init__(self, cost=-1, parent=None, position=[0, 0]):
        self.position = position
        self.cost = cost

        self.parent = parent
        self.children = []
        
        self.g = 0
        self.h = 0
        self.f = 0

    def __str__(self):
        return str(self.position) + " f: " + str(self.f) + " c: " + str(self.cost)

    def __eq__(self, otherNode):
        return self.position == otherNode.position

    def __gt__(self, otherNode):
        return self.f > otherNode.f

    def computeF(self, goal):
        x = abs(goal.position[1] - self.position[1])
        y = abs(goal.position[0] - self.position[0])
        self.h = x + y
        self.f = self.g + self.h

#Orders the list of nodes 
class Nodestack():

    def __init__(self):
        self.list = []

    def __len__(self):
        return len(self.list)

    def __contains__(self, node):
        return node in self.list

    def push(self, node: Node):
        self.list.append(node)
        self.list.sort()

    def pop(self):
        return self.list.pop(0)


#Checks if two nodes are next to eachother
def nextNodes(a: Node, b: Node):
    down_up = (a.position[0] == b.position[0]) and ((
        a.position[1] == b.position[1] + 1) or (a.position[1] == b.position[1] - 1))

    left_right = (a.position[1] == b.position[1]) and ((
        a.position[0] == b.position[0] + 1) or (a.position[0] == b.position[0] - 1))
    return (down_up or left_right)

#Adds all the successors of a node to a list
def generate_childnodes(currentnode, nodes):
    for node in nodes:
        if node.cost > -1:  
            if nextNodes(currentnode, node):
                currentnode.children.append(node)

#Calculate the weight between a parent and a child node and attach them together 
def attach_and_eval(parent: Node, child: Node, goal):
    child.parent = parent
    child.g = parent.g + child.cost
    child.computeF(goal)


#Runs the A* algorithm


def astar(start_node, goal_node, nodes):

    open_stack = Nodestack()
    closed_stack = []
    open_stack.push(start_node)

    while True:
        if not open_stack:
            return False

        current_node = open_stack.pop()
        closed_stack.append(current_node)

        if current_node == goal_node:
            return True

        generate_childnodes(current_node, nodes)

        for child_node in current_node.children:
            if child_node not in open_stack and child_node not in closed_stack:
                attach_and_eval(current_node, child_node, goal_node)
                open_stack.push(child_node)

            elif current_node.g + child_node.cost < child_node.g:
                attach_and_eval(current_node, child_node, goal_node)
                if child_node in closed_stack:
                    improve_path(child_node)


#Goes downwards in the tree and updates g-values and parents 
def improve_path(parent_node: Node):
    for child_node in parent_node.children:
        if parent_node.g + child_node.cost < child_node.g:
            child_node.parent = parent_node
            child_node.g = parent_node.g + child_node.cost
            improve_path(child_node)

#Sets up the inital state for solving the taskNums used in main method when taking in a taskNum number 
def initial_state(taskNum):

    map_obj = Map.Map_Obj(taskNum)
    start_pos = map_obj.start_pos
    goal_pos = map_obj.goal_pos
    mapPath = map_obj.fill_critical_positions(taskNum)[3]
    board = map_obj.read_map(mapPath)[0]
    start_node = None
    goal_node = None
    nodes = []

    for y in range(len(board)):
        for x in range(len(board[0])):
            node = Node(cost=board[y][x], position=[y, x])
            nodes.append(node)
            if node.position == start_pos:
                start_node = node
            elif node.position == goal_pos:
                goal_node = node

    return start_node, goal_node, nodes, board


def main():
    taskNum = int(input("Velg et oppgavenummer"))

    start_node, goal_node, nodes, board = initial_state(taskNum)

    if astar(start_node, goal_node, nodes):
        node = goal_node
        while node != start_node:
            board[node.position[0]][node.position[1]] = 7
            node = node.parent

    board[start_node.position[0]][start_node.position[1]] = 5
    board[goal_node.position[0]][goal_node.position[1]] = 6
    Map.Map_Obj.show_map(board)


if __name__ == '__main__':
    main()