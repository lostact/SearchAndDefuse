# -*- coding: utf-8 -*-

# python imports
from __future__ import print_function, division
import random
from collections import deque
import time,os

# chillin imports
from chillin_client import RealtimeAI

# project imports
import extensions
from cluster import kmeans

from ks.models import (World, Police, Terrorist, Bomb, Position, Constants, ESoundIntensity, ECell, EAgentStatus)
from ks.commands import DefuseBomb, PlantBomb, Move, ECommandDirection


class AI(RealtimeAI):
    def __init__(self, world):
        super(AI, self).__init__(world)
        self.done = False
    def initialize(self):
        start = time.time()
        os.system("printf '\033c'")
        print('initialize')

        # AI constants:
        self.DIRECTIONS = [
            ECommandDirection.Up,
            ECommandDirection.Right,
            ECommandDirection.Down,
            ECommandDirection.Left,
        ]

        self.DIR_TO_POS = {
            ECommandDirection.Up:    (-1, +0),
            ECommandDirection.Right: (+0, +1),
            ECommandDirection.Down:  (+1, +0),
            ECommandDirection.Left:  (+0, -1),
        }

        self.BOMBSITES_ECELL = [
            ECell.SmallBombSite,
            ECell.MediumBombSite,
            ECell.LargeBombSite,
            ECell.VastBombSite,
        ]

        self.BOMBSITE_COEFFICIENT = {
            ECell.SmallBombSite: self.world.constants.score_coefficient_small_bomb_site,
            ECell.MediumBombSite: self.world.constants.score_coefficient_medium_bomb_site,
            ECell.LargeBombSite: self.world.constants.score_coefficient_large_bomb_site,
            ECell.VastBombSite: self.world.constants.score_coefficient_vast_bomb_site,
        }

        self.sound_kinds = [ESoundIntensity.Strong,ESoundIntensity.Normal,ESoundIntensity.Weak]

        # agent constants:
        self.unreachable_bscore_coefficient = 2
        self.bombsite_size_coefficient = -1
        # self.unreachable_distance_power = 0.125
        self.failed_bombsite_coefficient = -10
        if self.my_side == 'Police':
            my_agents = self.world.polices
        else:
            my_agents = self.world.terrorists

        # bombsite dictionaries:
        self.bombsites = {}
        self.unreachable_bombsites = []
        self.start_pos = my_agents[0].position
        is_police = (self.my_side == 'Police')
        for row,line in enumerate(self.world.board):
            for col,cell in enumerate(line):
                if cell not in [ECell.Wall,ECell.Empty]:
                    distance, path = self._a_star(self.start_pos,(row,col),agent_block=False)
                    if path:
                        # KEYS:
                        # status key: 0<: clear for sure, 0: unkown, 1: bomb is possible, 2: bomb for sure 3: no time to defuse
                        # task key: 0 = nothing, 1 = going to act, 2 = acting 
                        # act means defuse or plant
                        self.bombsites[(row,col)] = {'size':cell,'initial_distance':distance, 'status':0,'task': 0,'bscore': 0, 'failed':0,'agent':None,'ert':-1}
                    else:
                        self.unreachable_bombsites.append((row,col))
        # ai variables:
        if self.my_side == 'Police':
            bombsite_positions = sorted(self.bombsites, key = lambda x: self.bombsites[x]['initial_distance'])
            self.clusters, self.cluster_centers = kmeans(bombsite_positions,len(my_agents))
            print(self.clusters, self.cluster_centers)
            self.cluster_index = [0] * len(my_agents)
            self.checked_bombsites = [[] for i in range(len(my_agents))]
            self.target_bombsites = [None] * len(my_agents)
            for i,cluster in enumerate(self.clusters):
                self.target_bombsites[i] = cluster[0]
        else:
            self.last_bombs = set([])
            self.heard_sound_count = [0] * len(my_agents)
            self.target_bombsites = [None] * len(my_agents)
            self.last_position = [[]] * len(my_agents)
            self.last_distance = [0] * len(my_agents)
            self.gir_count = [0] * len(my_agents)
            for agent in my_agents:
                self.last_position[agent.id] = agent.position

        for unreachable_bombsite in self.unreachable_bombsites:
            _, path, bypassed_bombsites = self._a_star(self.start_pos,unreachable_bombsite,agent_block=False,bombsite_block=False)
            bypassed_bombsite = bypassed_bombsites[-1]
            unreachable_size = self.world.board[unreachable_bombsite[0]][unreachable_bombsite[1]]
            self.bombsites[bypassed_bombsite]['bscore'] += (self.BOMBSITE_COEFFICIENT[unreachable_size] * self.unreachable_bscore_coefficient)
        self.last_scores = self.world.scores
        

        # timing:
        end = time.time()
        print('initialize time',end - start)

        #debug:
        print(self.unreachable_bombsites)
    def decide(self):
        start = time.time()
        print('decide' + ' ' + str(self.current_cycle))
        if self.my_side == 'Police':
            my_agents = self.world.polices
            # return 0
        else:
            my_agents = self.world.terrorists
        # ignore dead agents
        alive_agents = []
        for agent in my_agents:
            if agent.status == EAgentStatus.Dead:
                if self.my_side == 'Terrorist':
                    if self.target_bombsites[agent.id]:
                        if self.target_bombsites[agent.id] in self.bombsites:
                            self.bombsites[self.target_bombsites[agent.id]]['task'] = 0
                            self.bombsites[self.target_bombsites[agent.id]]['status'] = -1
                        self.target_bombsites[agent.id] = None
            else:
                alive_agents.append(agent)
        # update bombsites if terrorists score is changed:
        exploded_bombsites = []
        if self.last_scores['Terrorist'] != self.world.scores['Terrorist']:
            for bombsite in self.bombsites:
                if self.world.board[bombsite[0]][bombsite[1]] == ECell.Empty:   
                    exploded_bombsites.append(bombsite)
            for bombsite in exploded_bombsites:
                del self.bombsites[bombsite]
            if exploded_bombsites:
                print(exploded_bombsites)
                updated,unreachables,bypassed_bombsites_list = False, [], []
                for i,bombsite in enumerate(self.unreachable_bombsites):
                    distance, path, bypassed_bombsites = self._a_star(self.start_pos,bombsite,agent_block=False,bombsite_block=False)
                    if not bypassed_bombsites:
                        self.bombsites[bombsite] = {'size':self.world.board[bombsite[0]][bombsite[1]],'initial_distance':distance, 'status':-2,'task':0,'bscore':0, 'failed':0, 'agent':None,'ert':-1}
                        updated = True
                        print('unreachable opened:',bombsite,self.bombsites[bombsite])
                    else:
                        unreachables.append(bombsite)
                        bypassed_bombsites_list.append((bombsite,bypassed_bombsites))
                self.unreachable_bombsites = unreachables
                if updated:
                    for bombsite in self.bombsites:
                        self.bombsites[bombsite]['bscore'] = 0
                    for bombsite, bypassed_bombsites in bypassed_bombsites_list:
                        bypassed_bombsite = bypassed_bombsites[-1]
                        unreachable_size = self.world.board[bombsite[0]][bombsite[1]]
                        self.bombsites[bypassed_bombsite]['bscore'] += (self.BOMBSITE_COEFFICIENT[unreachable_size] * self.unreachable_bscore_coefficient)
                if self.my_side == 'Police':
                    bombsite_positions = sorted(self.bombsites, key = lambda x: self.bombsites[x]['initial_distance'])
                    self.clusters, self.cluster_centers = kmeans(bombsite_positions, len(my_agents))
                    self.cluster_index = [0] * len(my_agents)
                    print(self.clusters,self.cluster_centers)
        # prevent passing by exploding bombs
        for bomb in self.world.bombs:
            if bomb.explosion_remaining_time == 1:
                for neighbor in  bomb.position.get_neighbours(self.world):
                    self.world.board[neighbor.y][neighbor.x] = ECell.Wall

        if self.my_side == 'Terrorist':
            # update bombsites if bomb(s) is defused
            if self.last_scores['Police'] != self.world.scores['Police']:
                bomb_count = len(self.world.bombs)
                for last_bomb in self.last_bombs:
                    for index,bomb in enumerate(self.world.bombs):
                        if bomb.position == last_bomb.position:
                            break
                        elif index == bomb_count - 1:
                            bombsite_position = self._position_to_tuple(last_bomb.position)
                            if bombsite_position in self.bombsites:
                                self.bombsites[bombsite_position]['status'] = -1
                                self.bombsites[bombsite_position]['failed'] += 2
            self.last_bombs = self.world.bombs


            # choose nearest agent for each bombsite if bombsites number is smaller than agents number
            not_planted_bombsites = 0
            for bombsite in self.bombsites:
                if self.bombsites[bombsite]['status'] != 2:
                    not_planted_bombsites += 1
            if not_planted_bombsites < len(alive_agents):
                for agent_id,agent in enumerate(my_agents):
                    if agent.planting_remaining_time == -1:
                        self.target_bombsites[agent_id] = None

                for bombsite in self.bombsites:
                    if self.bombsites[bombsite]['status'] != 2:
                        min_distance = 1000
                        for agent in alive_agents:
                            if not self.target_bombsites[agent.id]:
                                distance, _ = self._a_star(agent.position,bombsite)
                                if distance < min_distance:
                                    best_agent = agent.id
                                    min_distance = distance
                        self.bombsites[bombsite]['task'] = 1
                        self.target_bombsites[best_agent] = bombsite
            print('targets:',self.target_bombsites)
        else:# police
            # update bombsites status according to sounds and visions
            for bombsite in self.bombsites:
                if self.bombsites[bombsite]['status'] < 0:
                    self.bombsites[bombsite]['status'] += 1
            in_vision_bombs = []
            for bomb in self.world.bombs:
                bombsite = self._position_to_tuple(bomb.position)
                self.bombsites[bombsite]['brt'] = bomb.explosion_remaining_time
                in_vision_bombs.append(bombsite)
            for agent in alive_agents:
                if agent.defusion_remaining_time == -1:
                    for bombsite in self.bombsites:
                        if self.bombsites[bombsite]['status'] >= 0:
                            if self._heuristic(bombsite,agent.position) <= self.world.constants.police_vision_distance:
                                if not bombsite in in_vision_bombs:
                                    if self._heuristic(bombsite,agent.position) == 1:
                                        self.bombsites[bombsite]['status'] = -1 * self.world.constants.bomb_planting_time - 1
                                    else:
                                        self.bombsites[bombsite]['status'] = -2
                                    self.bombsites[bombsite]['task'] = 0
                                else:
                                    self.bombsites[bombsite]['status'] = 2
                sound_counts = {ESoundIntensity.Weak:0,ESoundIntensity.Normal:0,ESoundIntensity.Strong:0}
                in_range_sites = {ESoundIntensity.Weak:[],ESoundIntensity.Normal:[],ESoundIntensity.Strong:[]}
                for sound in agent.bomb_sounds:
                    sound_counts[sound] += 1
                for bombsite in self.bombsites:
                    if self._heuristic(bombsite,agent.position) <= self.world.constants.sound_ranges[ESoundIntensity.Weak]:
                        distance, _ = self._a_star(agent.position, bombsite, not_valid_ecells=[ECell.Wall],agent_block=False)
                        if distance:
                            minimum_distance = self.world.constants.police_vision_distance
                            for sound_kind in self.sound_kinds:
                                if distance <= self.world.constants.sound_ranges[sound_kind] and distance > minimum_distance:
                                    if sound_counts[sound_kind]:
                                        status = self.bombsites[bombsite]['status']
                                        in_range_sites[sound_kind].append((bombsite,status,distance))
                                    else:
                                        self.bombsites[bombsite]['status'] = -2
                                        if self.target_bombsites[agent.id] == bombsite:
                                            self.checked_bombsites[agent.id].append(bombsite)
                                    break
                                minimum_distance = self.world.constants.sound_ranges[sound_kind]
                for sound_kind in self.sound_kinds:
                    if sound_counts[sound_kind] > 0:
                        possibles,sures = [], []
                        for index,(bombsite,status,distance) in enumerate(in_range_sites[sound_kind]):
                            if status >= 2:
                                sures.append(bombsite)
                            elif status >= 0:
                                possibles.append(bombsite)
                        if len(sures) == sound_counts[sound_kind]:
                            for bombsite in possibles:
                                self.bombsites[bombsite]['status'] = -2
                        else:
                            status = int((len(sures) + len(possibles)) <= sound_counts[sound_kind]) + 1
                            for bombsite in possibles:
                                self.bombsites[bombsite]['status'] = status
                                # print(agent.id,'at',agent.position,bombsite,status,in_range_sites)
            for bombsite in self.bombsites:
                if self.bombsites[bombsite]['ert'] > -1:
                    self.bombsites[bombsite]['ert'] -= 1
                elif self.bombsites[bombsite]['status'] == 2:
                    self.bombsites[bombsite]['ert'] = self.world.constants.bomb_explosion_time

        self.last_scores = self.world.scores
        # print(self.bombsites)
        for agent in alive_agents:
            if self.my_side == 'Police':
                bombsite_direction = self._find_bombsite_direction(agent)
                doing_bomb_operation = (agent.defusion_remaining_time != - 1)
                if doing_bomb_operation: # defusing
                    bombsite_position = self._sum_pos_tuples(self.DIR_TO_POS[bombsite_direction],(agent.position.y,agent.position.x))
                    self._agent_print(agent.id, 'Continuing bomb operation')
                    self.bombsites[bombsite_position]['status'] = 2
                    if agent.defusion_remaining_time == 1:
                        self.bombsites[bombsite_position]['status'] =  -1 * self.world.constants.bomb_planting_time - 1
                        self.bombsites[bombsite_position]['task'] = 0
                    continue
                if bombsite_direction:
                    try:
                        multiple = len(bombsite_direction)
                    except:
                        multiple = False
                    if multiple:
                        bombsite_direction = bombsite_direction[0]
                    bombsite_position = self._sum_pos_tuples(self.DIR_TO_POS[bombsite_direction],(agent.position.y,agent.position.x))
                    if self.target_bombsites[agent.id] == bombsite_position or multiple:
                        for bomb in self.world.bombs:
                            if self._position_to_tuple(bomb.position) == bombsite_position:
                                if bomb.explosion_remaining_time < self.world.constants.bomb_defusion_time:
                                    has_time = False
                                else:
                                    has_time = True
                                break
                        if has_time:
                            self._agent_print(agent.id, 'Starting bomb operation')
                            self.defuse(agent.id, bombsite_direction)
                            self.bombsites[bombsite_position]['task'] = 2
                            continue
                        else:
                            # self.scape_bombsite(agent)
                            self._agent_print(agent.id, 'direction: Ignoring bomb due to lack of time :(')
                            self.bombsites[bombsite_position]['status'] = 3
                            self.bombsites[bombsite_position]['task'] = 1
                            self.bombsites[bombsite_position]['agent'] = -1
                if self.world.bombs and not bombsite_direction: # bomb in vision:
                    found = False
                    for bomb in self.world.bombs:
                        bombsite_position = self._position_to_tuple(bomb.position)
                        if self.bombsites[bombsite_position]['task'] < 2 and self.bombsites[bombsite_position]['status'] < 3 and self.target_bombsites[agent.id] == bombsite_position and self._heuristic(agent.position,bombsite_position) <= self.world.constants.police_vision_distance:
                            distance, path = self._a_star(agent.position, bomb.position)
                            time_needed = distance + self.world.constants.bomb_defusion_time
                            if time_needed < bomb.explosion_remaining_time:
                                found = True
                                self.bombsites[bombsite_position]['task'] = 1
                                self.bombsites[bombsite_position]['agent'] = agent.id
                                self.path_move(agent,path)
                                self._agent_print(agent.id, 'Going to defuse.')
                                break
                                # print('time status:',time_needed,bomb.explosion_remaining_time)
                            else:
                                self._agent_print(agent.id, 'Ignoring bomb due to lack of time :(')
                                self.bombsites[bombsite_position]['task'] = 1
                                self.bombsites[bombsite_position]['agent'] = -1
                                self.bombsites[bombsite_position]['status'] = 3

                    if found:
                        continue
                # patrol:
                cluster = self.clusters[agent.id]

                if cluster:
                    best_bombsite, (best_distance, best_path) = self.best_bombsite_patrol(agent)
                    if best_path:
                        if len(best_path) > 1:
                            self.path_move(agent, best_path)
                else:
                    # len(bombsites) < len(agents)
                    pass
            else: # terrorist
                if self.last_position[agent.id] == agent.position:
                    self.gir_count[agent.id] += 1
                else:
                    self.gir_count[agent.id] = 0
                self.last_position[agent.id] = agent.position
                bombsite_direction = self._find_bombsite_direction(agent)
                doing_bomb_operation = agent.planting_remaining_time != -1
                threatened = False
                if ESoundIntensity.Strong in agent.footstep_sounds:
                    self.heard_sound_count[agent.id] += 1
                else:
                    self.heard_sound_count[agent.id] = 0
                if doing_bomb_operation:
                    try:
                        multiple = len(bombsite_direction)
                    except:
                        multiple = False
                    if multiple:
                        bombsite_direction = bombsite_direction[0]
                    bombsite_position = self.target_bombsites[agent.id]
                    if self.heard_sound_count[agent.id] >= (self.world.constants.terrorist_vision_distance - self.world.constants.police_vision_distance):
                        threatened = True
                        # self.heard_sound_count[agent.id] = 0
                    if not threatened:
                        self.gir_count[agent.id] = 0
                        if agent.planting_remaining_time <= 1:
                            self._agent_print(agent.id, 'Finishing bomb operation')
                            self.bombsites[bombsite_position]['task'] = 0
                            self.bombsites[bombsite_position]['status'] = 2
                            self.target_bombsites[agent.id] = None
                        else:
                            self._agent_print(agent.id, 'Continuing bomb operation')
                            self.bombsites[bombsite_position]['task'] = 2
                    else:
                        self._agent_print(agent.id, 'I swear I heard police footsteps, time to look around or maybe scape')
                        if len(self._empty_directions(agent.position)) == 1:
                            self.scape_bombsite(agent)
                        else:
                            self.move(agent.id, agent.position.direction_to(Position(bombsite_position[1],bombsite_position[0])))
                        self.bombsites[bombsite_position]['task'] = 1
                    continue
                if bombsite_direction:
                    bombsite_position = self._sum_pos_tuples(self.DIR_TO_POS[bombsite_direction],(agent.position.y,agent.position.x))
                    threatening_polices = []
                    for police in self.world.polices:
                        distance = self._heuristic(police.position,agent.position)
                        if distance <= self.world.constants.terrorist_vision_distance and police.status == EAgentStatus.Alive:
                            threatening_polices.append(police)
                    if threatening_polices:
                        self.scape_polices(agent,threatening_polices)
                        threatened = True
                        self.bombsites[bombsite_position]['failed'] += 1
                        self._agent_print(agent.id, "I see police(s). I can always plant, Now I must Scape.")
                        continue
                    if not threatened and self.bombsites[bombsite]['task'] != 2:
                        if bombsite_position == self.target_bombsites[agent.id] and self.bombsites[bombsite_position]['failed'] <= 5:
                            self._agent_print(agent.id, "Starting bomb operation, I don't see any polices.")
                            self.plant(agent.id, bombsite_direction)
                            self.bombsites[bombsite_position]['task'] = 2
                            continue
                # go to best bombsite
                if self.gir_count[agent.id] >= 4:
                    self._agent_print(agent.id, "Doing random move because GIR KARDAM !!!")
                    self.move(agent.id, random.choice(self._empty_directions(agent.position)))
                    continue
                best_bombsite, best_distance, best_path = self.best_bombsite_plant(agent)
                if len(best_path) > 1: # there IS a possible bombsite
                    self.bombsites[best_bombsite]['task'] = 1
                    self.target_bombsites[agent.id] = best_bombsite
                    self.last_distance[agent.id] = best_distance
                    self._agent_print(agent.id, 'Going to bombsite.')
                    self.path_move(agent,best_path)
                else: # no bombsites left to plant
                    threatening_polices = []
                    for police in self.world.polices:
                        if self._heuristic(agent.position,police.position) <= self.world.constants.police_vision_distance + 1:
                            threatening_polices.append(police)
                    if threatening_polices:
                        self.scape_polices(agent,threatening_polices)
                        self._agent_print(agent.id, 'Nothing to do, Scaping police(s).')
                        continue
                    else:
                        bombsite_position = self._find_bombsite_direction(agent,possible_only=False)
                        if bombsite_position:
                            self.scape_bombsite(agent)
                            self._agent_print(agent.id, 'Nothing to do, Getting a safe distance with the last planted bomb.')
                            continue
                        else:
                            self._agent_print(agent.id, 'Nothing to do, waiting ZzZzZ...')



        # if self.my_side == 'Police':
        #     for agent_1 in patrol_moves:
        #         for agent_2 in patrol_moves:
        #             if agent_1 != agent_2 and self._position_to_tuple(agent_1.position) == patrol_moves[agent_2] and self._position_to_tuple(agent_2.position) == patrol_moves[agent_1]:
        #                 self.cluster_index[agent_1.id],self.cluster_index[agent_2.id] = self.cluster_index[agent_2.id],self.cluster_index[agent_1.id]
        #                 bombsite_position_1 = self.bombsite_positions[self.cluster_index[agent_1.id]]
        #                 bombsite_position_2 = self.bombsite_positions[self.cluster_index[agent_2.id]]
        #                 _, path_1 = self._a_star(agent_1.position, bombsite_position_1)
        #                 _, path_2 = self._a_star(agent_2.position, bombsite_position_1)
        #                 patrol_moves[agent_1] = path_1[1]
        #                 patrol_moves[agent_2] = path_2[1]
        #     for agent in patrol_moves:
        #         if patrol_moves[agent]:
        #             self.position_move(agent,patrol_moves[agent])
        # else:
        #     print(self.heard_sound_count)
        if self.my_side == 'Police':
            print(self.target_bombsites)
            # print(self.bombsites)
            # for bombsite in self.bombsites:
            #     print(bombsite,'status:',self.bombsites[bombsite]['status'],'task:',self.bombsites[bombsite]['task'],end=',')
            # print()
        end = time.time()
        print('time:',end - start)
    def plant(self, agent_id, bombsite_direction):

        self.send_command(PlantBomb(id=agent_id, direction=bombsite_direction))
    def best_bombsite_patrol(self,agent):
        # print(agent.id,self.checked_bombsites[agent.id])
        best_score, best_bombsite, best_path, best_distance = 0, None, [], None
        cluster = self.clusters[agent.id]
        possible_bombsites  = []
        scores = {}
        possible_other_bombsites = []
        maximum_status = -10
        other_cluster = False
        last_target_status = 3
        if self.target_bombsites[agent.id] in self.bombsites:
            last_target_status = self.bombsites[self.target_bombsites[agent.id]]['status']
        if last_target_status == 3:
            last_target_status = -1
        for bombsite in cluster:
            status = self.bombsites[bombsite]['status']
            if status > last_target_status and status < 3:
                closer = self.bombsites[bombsite]['task'] != 2
                if self.bombsites[bombsite]['task'] == 1 and self.bombsites[bombsite]['agent'] != agent.id:
                    other_distance = self._a_star(self.world.polices[self.bombsites[bombsite]['agent']].position,bombsite)
                    my_distance = self._a_star(agent.position,bombsite)
                    if my_distance >= other_distance:
                        closer = False
                if closer:
                    possible_bombsites.append(bombsite)
                    if status > maximum_status:
                        maximum_status = status
        if maximum_status < 2:
            for bombsite in self.bombsites:
                if self.bombsites[bombsite]['status'] == 2:
                    closer = self.bombsites[bombsite]['task'] != 2
                    if self.bombsites[bombsite]['task'] == 1 and self.bombsites[bombsite]['agent'] != agent.id:
                        other_distance = self._a_star(self.world.polices[self.bombsites[bombsite]['agent']].position,bombsite)
                        my_distance = self._a_star(agent.position,bombsite)
                        if my_distance >= other_distance:
                            closer = False
                    if closer:
                        possible_other_bombsites.append(bombsite)
            if possible_other_bombsites:
                possible_bombsites = possible_other_bombsites
                other_cluster = True
        for bombsite in possible_bombsites:
            if bombsite not in self.checked_bombsites[agent.id]:
                distance, path = self._a_star(agent.position, bombsite)
                if len(path) > 1:
                    score = (((self.bombsites[bombsite]['status'] - 1) * 1000) + (self.BOMBSITE_COEFFICIENT[self.bombsites[bombsite]['size']] * 5) + self.bombsites[bombsite]['bscore'] * 20) * (self.bombsites[bombsite]['ert'] / (distance + self.world.constants.bomb_defusion_time) - 1)
                    if score > 0:
                        scores[bombsite] = score
                    # elif status == 2 and not other_cluster:
                    #     min_distance = 1000
                    #     my_agents = self.world.polices
                    #     closest_agent = agent
                    #     for new_agent in my_agents:
                    #         distance, path = self._a_star(bombsite,new_agent.position,agent_block=False)
                    #         if path:
                    #             if distance < min_distance:
                    #                 min_distance = distance
                    #                 closest_agent = new_agent
                    #     if closest_agent != agent:
                    #         print('dumped bombsite',bombsite,'from',agent.id,'to',closest_agent.id)
                    #         self.clusters[closest_agent.id].append(bombsite)
                    #         self.clusters[agent.id].remove(bombsite)
                    #     else:
                    #         self.bombsites[bombsite]['status'] = 3
        if scores:
            print(agent.id,scores)
            best_bombsite = max(scores, key=scores.get)
        else:
            tries = 0
            cluster_size = len(cluster)
            if self.cluster_index[agent.id] < cluster_size:
                best_bombsite = cluster[self.cluster_index[agent.id]]
            else:
                best_bombsite = cluster[0]
            while((self.bombsites[best_bombsite]['status'] < 0 and tries < cluster_size) or self.bombsites[best_bombsite]['status'] == 3):
                tries += 1
                if self.cluster_index[agent.id] < len(cluster) - 1:
                    self.cluster_index[agent.id] += 1
                else:
                    self.cluster_index[agent.id] = 0
                best_bombsite = cluster[self.cluster_index[agent.id]]
            self.checked_bombsites[agent.id] = []
        last_bombsite = self.target_bombsites[agent.id]
        if last_bombsite and last_bombsite in self.bombsites:
            self.bombsites[last_bombsite]['task'] = 0
            self.bombsites[best_bombsite]['agent'] = -1
        self.target_bombsites[agent.id] = best_bombsite
        self.bombsites[best_bombsite]['task'] = 1
        self.bombsites[best_bombsite]['agent'] = agent.id
        return best_bombsite, self._a_star(agent.position,best_bombsite)
    def best_bombsite_plant(self,agent):
        best_score, best_bombsite, best_path, best_distance = -10000, None, [], None
        last_bombsite, last_path =  self.target_bombsites[agent.id], None
        blocked = True
        if last_bombsite:
            new_distance, new_path = self._a_star(agent.position,last_bombsite)
            if new_path and self.last_distance[agent.id]:
                if (new_distance - self.last_distance[agent.id]) < 4 or self.last_distance[agent.id] <= 2:
                    blocked = False
            if blocked:
                print(self.target_bombsites[agent.id],last_bombsite,self.last_distance[agent.id])
                self.bombsites[last_bombsite]['failed'] += 1
                print('agent',agent.id,'blocked from reaching',last_bombsite,'with',self.bombsites[last_bombsite]['failed'],'failed attempts')
        if not last_bombsite or not last_path or blocked:
            for bombsite in self.bombsites:
                if self.bombsites[bombsite]['status'] != 2 and (self.bombsites[bombsite]['task'] < 1 or bombsite == self.target_bombsites[agent.id]):
                    distance, path = self._a_star(agent.position,bombsite)
                    if len(path) > 1:
                        score = self.bombsites[bombsite]['bscore'] + (self.bombsites[bombsite]['failed'] * self.failed_bombsite_coefficient) + (self.BOMBSITE_COEFFICIENT[self.bombsites[bombsite]['size']] * self.bombsite_size_coefficient) - distance
                        if score > best_score:
                            best_bombsite = bombsite
                            best_path = path
                            best_score = score
                            best_distance = distance
            if last_bombsite and not last_path:
                self.bombsites[last_bombsite]['task'] = 0
        else:
            best_bombsite, best_distance, best_path = last_bombsite, last_distance, last_path
        return best_bombsite, best_distance, best_path
    def path_move(self,agent,path):
        direction = agent.position.direction_to(Position(x=path[1][1],y=path[1][0]))
        self.move(agent.id, direction)
        agent.position = Position(x=path[1][1],y=path[1][0])
        # self._agent_print(agent.id, 'Path Move {}'.format(direction))
    def position_move(self,agent,position):
        # try:
            direction = agent.position.direction_to(Position(x=position[1],y=position[0]))
            self.move(agent.id, direction)
            agent.position = Position(x=position[1],y=position[0])
            # agent.position = Position(x=path[1][1],y=path[1][0])
            # self._agent_print(agent.id, 'Path Move {}'.format(direction))
        # except:
        #     print('position move error:',agent,position)
    def scape_polices(self,agent,polices):
        agent_positions = []
        for terrorist in self.world.terrorists:
                agent_positions.append(terrorist.position)
        
        min_score = 0
        best_neighbor = None
        for police in polices:
            min_score += self._heuristic(agent.position,police.position)
        for neighbor_position in agent.position.get_neighbours(self.world):
            if self.world.board[neighbor_position.y][neighbor_position.x] != ECell.Empty or neighbor_position in agent_positions:
                continue
            score = 0
            for police in polices:
                score += self._heuristic(neighbor_position,police.position)
            if score > min_score:
                min_score = score
                best_neighbor = neighbor_position
        if best_neighbor:
            direction = agent.position.direction_to(best_neighbor)  
            self.move(agent.id, direction)
            agent.position = best_neighbor
            return True
        return False
    def scape_bombsite(self,agent):
        agent_positions = []
        for terrorist in self.world.terrorists:
                agent_positions.append(terrorist.position)
        for neighbor_position in agent.position.get_neighbours(self.world):
            if self.world.board[neighbor_position.y][neighbor_position.x] != ECell.Empty or neighbor_position in agent_positions:
                continue
            else:
                direction = agent.position.direction_to(neighbor_position)
                self.move(agent.id, direction)
                agent.position = neighbor_position
                return True
        return False
    def defuse(self, agent_id, bombsite_direction):
        self.send_command(DefuseBomb(
            id=agent_id, direction=bombsite_direction))
    def move(self, agent_id, move_direction):

        self.send_command(Move(id=agent_id, direction=move_direction))
    def _empty_directions(self, position):
        empty_directions = []

        for direction in self.DIRECTIONS:
            pos = self._sum_pos_tuples((position.y, position.x), self.DIR_TO_POS[direction])
            if self.world.board[pos[0]][pos[1]] == ECell.Empty:
                empty_directions.append(direction)
        return empty_directions
    def _find_bombsite_direction(self, agent,possible_only=True):
        directions = []
        for direction in self.DIRECTIONS:
            pos = self._sum_pos_tuples((agent.position.y, agent.position.x), self.DIR_TO_POS[direction])
            if self.world.board[pos[0]][pos[1]] in self.BOMBSITES_ECELL:
                has_bomb = self._has_bomb(pos)
                if ((self.my_side == 'Police' and has_bomb) or (self.my_side == 'Terrorist' and not has_bomb)):
                    directions.append(direction)
                elif not possible_only:
                    return pos
        if len(directions) == 1:
            return directions[0]
        elif len(directions) > 1:
            return directions
        return None
    def _has_bomb(self, position):
        for bomb in self.world.bombs:
            if position[0] == bomb.position.y and position[1] == bomb.position.x:
                return True
        return False
    def _sum_pos_tuples(self, t1, t2):
        t1,t2 = self._position_to_tuple(t1),self._position_to_tuple(t2)
        return (t1[0] + t2[0],t1[1] + t2[1])
    def _position_to_tuple(self,position):
        try:
            return (position.y,position.x)
        except:
            return position
    def _agent_print(self, agent_id, text):

        print('Agent[{}]: {}'.format(agent_id, text))
    def _reconstruct_path(self, came_from, current,bombsite_block=True):
        final_path = [current]
        distance = 0
        bypassed_bombsites = []
        while current in came_from:
            distance += 1
            current = came_from[current]
            if not bombsite_block:
                if self.world.board[current[0]][current[1]] in self.BOMBSITES_ECELL:
                    bypassed_bombsites.append(current)
            final_path.append(current)
        if not bombsite_block:
            return distance, list(reversed(final_path)), bypassed_bombsites
        return distance, list(reversed(final_path))
    def _check_valid(self,ecell,valid_ecells):
        if ecell in valid_ecells:
            return True
        return False
    def _check_not_valid(self,ecell,not_valid_ecells):
        if ecell not in not_valid_ecells:
            return True
        return False
    def _dist_between(self, goal,agent_positions=[],police_positions=[], bombsite_block = True):
        dist_between = 1

        min_distance = 1000
        for police_position in police_positions:
            distance = self._heuristic(goal,police_position)
            if distance < min_distance:
                min_distance = distance
        if min_distance <= self.world.constants.police_vision_distance + 1:
            dist_between += abs(min_distance - (self.world.constants.police_vision_distance + 2)) * 1000
        elif min_distance == self.world.constants.police_vision_distance + 2 and len(self._empty_directions(Position(y=goal[0],x=goal[1]))) == 1:
            dist_between += 2000
        if not bombsite_block:
            if self.world.board[goal[0]][goal[1]] != ECell.Wall and self.world.board[goal[0]][goal[1]] != ECell.Empty:
                dist_between += 100

        return dist_between
    def _heuristic(self, current, goal):
        current,goal = self._position_to_tuple(current),self._position_to_tuple(goal)
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])
    def _a_star(self, start_position, goal_position, valid_ecells=[ECell.Empty],not_valid_ecells=None,agent_block=True,bombsite_block=True):
        start = self._position_to_tuple(start_position)
        goal = self._position_to_tuple(goal_position)
        blocked_by_agent = False

        if not not_valid_ecells:
            check = self._check_valid
            ecells = valid_ecells
        else:
            check = self._check_not_valid
            ecells = not_valid_ecells

        if not bombsite_block:
            ecells = [ECell.Empty]
            for ecell in self.BOMBSITES_ECELL:
                ecells.append(ecell)
        police_positions = []
        agent_positions = []
        if agent_block:
            if self.my_side == 'Police':
                agents = self.world.polices
            else:
                agents = self.world.terrorists
                for police in self.world.polices:
                    if police.status == EAgentStatus.Alive:
                        police_positions.append(police.position)
            for agent in agents:
                if agent.status == EAgentStatus.Alive and self._position_to_tuple(agent.position) != start:
                    agent_positions.append(agent.position)

        closed_set = {}
        # key: position, value: f_score
        open_set = {start: self._heuristic(start, goal)}
        came_from = {}
        g_score = {start: 0}
        min_blocking_distance = 100000
        while open_set:
            current = min(open_set, key=open_set.get)
            if current == goal:
                return self._reconstruct_path(came_from, current, bombsite_block=bombsite_block)

            del open_set[current]
            closed_set[current] = True
            for neighbor_position in Position(x=current[1],y=current[0]).get_neighbours(self.world):
                neighbor = (neighbor_position.y,neighbor_position.x)
                if neighbor in closed_set:
                    continue
                if not check(self.world.board[neighbor[0]][neighbor[1]],ecells) and neighbor != goal:
                    continue
                if agent_block:
                    if neighbor_position in agent_positions:
                        if self._heuristic(goal,neighbor_position) < min_blocking_distance:
                            best_blocking_neighbor = neighbor
                            best_blocking_current = current
                            min_blocking_distance = self._heuristic(goal,neighbor_position) + self._dist_between(neighbor,agent_positions,police_positions)
                            blocked_by_agent = True
                        continue
                tentative_g_score = g_score[current] + self._dist_between(neighbor,agent_positions,police_positions, bombsite_block=bombsite_block)
                if tentative_g_score > 1000:
                    continue
                if neighbor in open_set and tentative_g_score >= g_score[neighbor]:
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                open_set[neighbor] = g_score[neighbor] + self._heuristic(neighbor, goal)
        if blocked_by_agent:
            came_from[best_blocking_neighbor] = best_blocking_current
            return self._reconstruct_path(came_from,best_blocking_neighbor)
        if bombsite_block:
            return None, []
        else:
            return None, [], []
    # def _dls(self, start_position, max_depth, goal_position=None, valid_ecells=[ECell.Empty]):
    #     start_position = self._position_to_tuple(start_position)
    #     goal_position = self._position_to_tuple(goal_position)
    #     if goal_position != None and goal_position == start_position:
    #         return 0, []

    #     positions = deque([start_position])
    #     depths = deque([0])
    #     visited = [start_position]

    #     while len(positions) > 0:
    #         pos = positions.popleft()
    #         depth = depths.popleft()

    #         if max_depth is None or depth < max_depth:
    #             # Check neighbours
    #             neighbours = pos.get_neighbours(self.world)
    #             for neighbour in neighbours:
    #                 # Check ecell
    #                 board_ecell = self.world.board[neighbour.y][neighbour.x]
    #                 if valid_ecells != None and self.world.board[neighbour.y][neighbour.x] not in valid_ecells:
    #                     continue
    #                 # Check visited
    #                 if neighbour in visited:
    #                     continue

    #                 # Update visited
    #                 visited.append((neighbour.y,neighbour.x))

    #                 # Check goal
    #                 if goal_position != None and neighbour == goal_position:
    #                     return depth + 1, visited

    #                 # Update queue
    #                 positions.append(neighbour)
    #                 depths.append(depth + 1)
    #     return visited
