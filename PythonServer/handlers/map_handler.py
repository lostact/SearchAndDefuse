# -*- coding: utf-8 -*-

# python imports
import json

# project imports
# from ..ks.models import World, ECell
import sys
sys.path.append('../')
from ks.models import World, ECell, Constants

GLOBAL_BOARD_WIDTH = 0
GLOBAL_BOARD_HEIGHT = 0


class MapHandler:

    def __init__(self, sides):
        self._sides = sides

    def load_map(self, config):
        map_config = json.loads(open((config['map']), "r").read())
        char_board = map_config['char_board']
        self.BOARD_WIDTH = len(char_board[0])
        self.BOARD_HEIGHT = len(char_board)

        world = World()
        world.width = self.BOARD_WIDTH
        world.height = self.BOARD_HEIGHT
        world.map_config = map_config
        world.config = config
        world.scores = {side: 0 for side in self._sides}
        world.bombs = []
        world.polices = []
        world.terrorists = []
        world.constants = Constants()
        world.constants.bomb_planting_time = map_config["constants"]["bomb_planting_time"]
        world.constants.bomb_defusion_time = map_config["constants"]["bomb_defusion_time"]
        world.constants.bomb_explosion_time = map_config["constants"]["bomb_explosion_time"]
        world.constants.bomb_planting_score = map_config["constants"]["bomb_planting_score"]
        world.constants.bomb_defusion_score = map_config["constants"]["bomb_defusion_score"]
        world.constants.bomb_explosion_score = map_config["constants"]["bomb_explosion_score"]
        world.constants.score_coefficient_small_bomb_site = map_config["constants"]["score_coefficient_small_bomb_site"]
        world.constants.score_coefficient_medium_bomb_site = map_config["constants"]["score_coefficient_medium_bomb_site"]
        world.constants.score_coefficient_large_bomb_site = map_config["constants"]["score_coefficient_large_bomb_site"]
        world.constants.score_coefficient_vast_bomb_site = map_config["constants"]["score_coefficient_vast_bomb_site"]
        world.constants.terrorist_vision_distance = map_config["constants"]["terrorist_vision_distance"]
        world.constants.terrorist_death_score = map_config["constants"]["terrorist_death_score"]
        world.constants.police_vision_distance = map_config["constants"]["police_vision_distance"]
        world.constants.max_cycles = map_config["constants"]["max_cycles"]
        world.board = [[ECell.Empty for _ in range(world.width)] for _ in range(world.height)]

        return world, char_board
