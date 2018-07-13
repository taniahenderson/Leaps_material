from .Agent import *


class AgentDemo(Agent):

    def __init__(self, maze_env):
        super(AgentDemo, self).__init__(maze_env)
        self.virtual = True
        self.maze_env = maze_env
        self.counter = 0

    def move(self):
        while self.look(0)[0] != 3:
            x = random.choice([0,1,2,3])
            if self.look(x)[0] != -1:
                self.step(x)

