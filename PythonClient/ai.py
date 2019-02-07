# -*- coding: utf-8 -*-

# python imports
import random
from collections import deque

# chillin imports
from chillin_client import RealtimeAI

# project imports
import extensions
from ks.models import (World, Police, Terrorist, Bomb, Position, Constants,
                       ESoundIntensity, ECell, EAgentStatus)
from ks.commands import DefuseBomb, PlantBomb, Move, ECommandDirection


class AI(RealtimeAI):

    def __init__(self, world):
        super(AI, self).__init__(world)
        self.done = False

    def initialize(self):
        print('initialize')

        self.DIRECTIONS = [
            ECommandDirection.Up,
            ECommandDirection.Right,
            ECommandDirection.Down,
            ECommandDirection.Left,
        ]

        self.DIR_TO_POS = {
            ECommandDirection.Up:    (+0, -1),
            ECommandDirection.Right: (+1, +0),
            ECommandDirection.Down:  (+0, +1),
            ECommandDirection.Left:  (-1, +0),
        }

        self.BOMBSITES_ECELL = [
            ECell.SmallBombSite,
            ECell.MediumBombSite,
            ECell.LargeBombSite,
            ECell.VastBombSite,
        ]

        self.VALID_WALK_ECELLS = [
            ECell.Empty,
        ]


    def decide(self):
        print('decide')

        my_agents = self.world.polices if self.my_side == 'Police' else self.world.terrorists
        for agent in my_agents:
            if agent.status == EAgentStatus.Dead:
                continue

            if agent.id == 0:
                _, visions = self._dls(agent.position, self.world.constants.police_vision_distance)

                path = self._a_star(agent.position, Position(x=21, y=1), self.VALID_WALK_ECELLS)
                if path is not None and len(path) >= 2: # one element is start position
                    direction = agent.position.direction_to(Position.from_tuple(path[len(path) - 2])) # last element is start position
                    self._agent_print(agent.id, 'Path Move {}'.format(direction))
                    if direction is not None:
                        self.move(agent.id, direction)

                continue

            doing_bomb_operation = agent.defusion_remaining_time != -1 if self.my_side == 'Police' else agent.planting_remaining_time != -1

            if doing_bomb_operation:
                self._agent_print(agent.id, 'Continue Bomb Operation')
                continue

            bombsite_direction = self._find_bombsite_direction(agent)
            if bombsite_direction == None:
                self._agent_print(agent.id, 'Random Move')
                self.move(agent.id, random.choice(self._empty_directions(agent.position)))
            else:
                self._agent_print(agent.id, 'Start Bomb Operation')
                if self.my_side == 'Police':
                    self.defuse(agent.id, bombsite_direction)
                else:
                    self.plant(agent.id, bombsite_direction)


    def plant(self, agent_id, bombsite_direction):
        self.send_command(PlantBomb(id=agent_id, direction=bombsite_direction))


    def defuse(self, agent_id, bombsite_direction):
        self.send_command(DefuseBomb(id=agent_id, direction=bombsite_direction))


    def move(self, agent_id, move_direction):
        self.send_command(Move(id=agent_id, direction=move_direction))


    def _empty_directions(self, position):
        empty_directions = []

        for direction in self.DIRECTIONS:
            pos = self._sum_pos_tuples((position.x, position.y), self.DIR_TO_POS[direction])
            if self.world.board[pos[1]][pos[0]] == ECell.Empty:
                empty_directions.append(direction)
        return empty_directions


    def _find_bombsite_direction(self, agent):
        for direction in self.DIRECTIONS:
            pos = self._sum_pos_tuples((agent.position.x, agent.position.y), self.DIR_TO_POS[direction])
            if self.world.board[pos[1]][pos[0]] in self.BOMBSITES_ECELL:
                has_bomb = self._has_bomb(pos)
                if (self.my_side == 'Police' and has_bomb) or (self.my_side == 'Terrorist' and not has_bomb):
                    return direction
        return None


    def _has_bomb(self, position):
        for bomb in self.world.bombs:
            if position[0] == bomb.position.x and position[1] == bomb.position.y:
                return True
        return False


    def _sum_pos_tuples(self, t1, t2):
        return (t1[0] + t2[0], t1[1] + t2[1])


    def _agent_print(self, agent_id, text):
        print('Agent[{}]: {}'.format(agent_id, text))


    def _reconstruct_path(self, came_from, current):
        final_path = [current]
        while current in came_from:
            current = came_from[current]
            final_path.append(current)
        return final_path


    # Return the path as list of tuples (x, y)
    def _a_star(self, start_position, goal_position, valid_ecells):
        start = start_position.to_tuple()
        goal = goal_position.to_tuple()

        closed_set = {}
        open_set = {start: self._heuristic(start, goal)} # key: position, value: f_score
        came_from = {}
        g_score = {start: 0}

        while len(open_set) > 0:
            current = min(open_set, key=open_set.get)
            if current == goal:
                return self._reconstruct_path(came_from, current)

            del open_set[current]
            closed_set[current] = True

            for neighbor_position in Position.from_tuple(current).get_neighbours(self.world):
                if self.world.board[neighbor_position.y][neighbor_position.x] not in valid_ecells:
                    continue

                neighbor = neighbor_position.to_tuple()
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + self._dist_between(current, neighbor)

                if neighbor in open_set and tentative_g_score >= g_score[neighbor]:
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                open_set[neighbor] = g_score[neighbor] + self._heuristic(neighbor, goal)

        return None


    # positions are in tuple form
    def _dist_between(self, current, goal):
        return 1


    # positions are in tuple form
    def _heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


    # depth limited search, return the depth and list of visited positions
    def _dls(self, start_position, max_depth, goal_position=None, valid_ecells=None):
        if goal_position is not None and goal_position == start_position:
            return 0, []

        positions = deque([start_position])
        depths = deque([0])
        visited = [start_position]

        while len(positions) > 0:
            pos = positions.popleft()
            depth = depths.popleft()

            if max_depth is None or depth < max_depth:
                # Check neighbours
                neighbours = pos.get_neighbours(self.world)
                for neighbour in neighbours:
                    # Check ecell
                    board_ecell = self.world.board[neighbour.y][neighbour.x]
                    if valid_ecells is not None and self.world.board[neighbour.y][neighbour.x] not in valid_ecells:
                        continue
                    # Check visited
                    if neighbour in visited:
                        continue

                    # Update visited
                    visited.append(neighbour)

                    # Check goal
                    if not goal_position is None and neighbour == goal_position:
                        return depth + 1, visited

                    # Update queue
                    positions.append(neighbour)
                    depths.append(depth + 1)

        return None, visited
