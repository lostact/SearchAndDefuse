# -*- coding: utf-8 -*-

# python imports
import math

# chillin imports
from chillin_server.gui.canvas_elements import ScaleType

# project imports
from ..ks.commands import *
from ..ks.models import *
from ..gui_events import GuiEventType


class GuiHandler:

    def __init__(self, world, sides, canvas, config):

        self._world = world
        self._sides = sides
        self._canvas = canvas
        self._config = config
        self._scale_factor = canvas.width / (self._world.width * config['cell_size'])
        self._scale_percent = math.ceil(self._scale_factor * 100)
        self._cell_size = math.ceil(config['cell_size'] * self._scale_factor)
        self._font_size = int(self._cell_size / 2)
        self._utils = GuiUtils(self._cell_size)
        self._img_refs = {side: {} for side in self._sides}
        self._img_refs['bombs'] = {"small": [], "medium": [], "large": [], "vast": []}

    def initialize(self):
        canvas = self._canvas
        config = self._config

        # Draw background
        background_ref = canvas.create_image('Background', 0, 0)
        canvas.edit_image(background_ref, scale_type=ScaleType.ScaleToWidth,
                          scale_value=canvas.width)

        self.angle = {
            ECommandDirection.Up.name: -90,
            ECommandDirection.Right.name: 180,
            ECommandDirection.Down.name: 90,
            ECommandDirection.Left.name: 0
        }

        self._initialize_board(canvas)

    def update(self, gui_events):
        moving_terrorists, moving_polices = [], []
        bombs_events = {"planting": [], "planted": [], "exploded": []}

        for event in gui_events:
            if event.type == GuiEventType.MovePolice:
                moving_polices.append(event.payload)
            elif event.type == GuiEventType.MoveTerrorist:
                moving_terrorists.append(event.payload)
            elif event.type == GuiEventType.PlantingBomb:
                bombs_events['planting'].append(event.payload)
            elif event.type == GuiEventType.PlantedBomb:
                bombs_events['planted'].append(event.payload)
            elif event.type == GuiEventType.ExplodeBomb:
                bombs_events['exploded'].append(event.payload)

        if (len(moving_terrorists) != 0) or (len(moving_polices) != 0):
            self._update_board_on_move(moving_terrorists, moving_polices)

        elif len(bombs_events['planting']) != 0:
            self._update_board_on_planting(bombs_events['planting'])

        elif len(bombs_events['planted']) != 0:
            self._update_board_on_planted(bombs_events['planted'])

        elif len(bombs_events['exploded']) != 0:
            self._update_board_on_explode(bombs_events['exploded'])

    def _update_board_on_move(self, moving_terrorists, moving_polices):
        for side in self._sides:
            moves = moving_polices if side == 'Police' else moving_terrorists

            for move in moves:
                canvas_pos = self._utils.get_canvas_position(move['agent_position'])
                # terrorist.angle = self.angle[EDirection.Left.name]
                self._canvas.edit_image(self._img_refs[side][move['agent_id']],
                                        canvas_pos['x'], canvas_pos['y'],
                                        center_origin=True)

    def _update_board_on_planting(self, bombs_planting):
        for bomb in bombs_planting:
            canvas_pos = self._utils.get_canvas_position(bomb['bomb_position'])
            board_cell = self._world.board[bomb['bomb_position'].y][bomb['bomb_position'].x]
            if board_cell == ECell.SmallBombSite:
                self._canvas.create_image('PlantingBomb', canvas_pos['x'], canvas_pos['y'],
                                                                                     center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.MediumBombSite:
                self._canvas.create_image('PlantingBomb', canvas_pos['x'], canvas_pos['y'],
                                                                                      center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.LargeBombSite:
                self._canvas.create_image('PlantingBomb', canvas_pos['x'], canvas_pos['y'],
                                                                                     center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.VastBombSite:
                self._canvas.create_image('PlantingBomb', canvas_pos['x'], canvas_pos['y'],
                                                                                    center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)

    def _update_board_on_planted(self, bombs_planted):
        for bomb in bombs_planted:
            canvas_pos = self._utils.get_canvas_position(bomb['bomb_position'])
            board_cell = self._world.board[bomb['bomb_position'].y][bomb['bomb_position'].x]
            if board_cell == ECell.SmallBombSite:
                self._canvas.create_image('PlantedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.MediumBombSite:
                self._canvas.create_image('PlantedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.LargeBombSite:
                self._canvas.create_image('PlantedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.VastBombSite:
                self._canvas.create_image('PlantedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)

    def _update_board_on_explode(self, bombs_exploded):
        for bomb in bombs_exploded:
            canvas_pos = self._utils.get_canvas_position(bomb['bomb_position'])
            board_cell = self._world.board[bomb['bomb_position'].y][bomb['bomb_position'].x]
            if board_cell == ECell.SmallBombSite:
                self._canvas.create_image('ExplodedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.MediumBombSite:
                self._canvas.create_image('ExplodedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.LargeBombSite:
                self._canvas.create_image('ExplodedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
            elif board_cell == ECell.VastBombSite:
                self._canvas.create_image('ExplodedBomb', canvas_pos['x'], canvas_pos['y'],
                                          center_origin=True, scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)

    def _initialize_board(self, canvas):
        for y in range(self._world.height):
            for x in range(self._world.width):
                cell = self._world.board[y][x]
                canvas_pos = self._utils.get_canvas_position(Position(x=x, y=y), center_origin=False)

                # Draw non-player cells
                if cell == ECell.Empty:
                    canvas.create_image('Empty', canvas_pos['x'], canvas_pos['y'],
                                        scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
                elif cell == ECell.Wall:
                    canvas.create_image('Wall', canvas_pos['x'], canvas_pos['y'],
                                        scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
                elif cell == ECell.SmallBombSite:
                    small_bomb_ref = canvas.create_image('SmallBomb', canvas_pos['x'], canvas_pos['y'],
                                                         scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
                    self._img_refs['bombs']['SmallSite'] = small_bomb_ref
                elif cell == ECell.MediumBombSite:
                    medium_bomb_ref = canvas.create_image('MediumBomb', canvas_pos['x'], canvas_pos['y'],
                                                          scale_type=ScaleType.ScaleToWidth,
                                                          scale_value=self._cell_size)
                    self._img_refs['bombs']['MediumSite'] = medium_bomb_ref
                elif cell == ECell.LargeBombSite:
                    large_bomb_ref = canvas.create_image('LargeBomb', canvas_pos['x'], canvas_pos['y'],
                                                         scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
                    self._img_refs['bombs']['LargeSite'] = large_bomb_ref
                elif cell == ECell.VastBombSite:
                    vast_bomb_ref = canvas.create_image('VastBomb', canvas_pos['x'], canvas_pos['y'],
                                                        scale_type=ScaleType.ScaleToWidth, scale_value=self._cell_size)
                    self._img_refs['bombs']['VastSite'] = vast_bomb_ref


        # Draw Agents
        for side in self._sides:
            agents = self._world.polices if side == 'Police' else self._world.terrorists

            for agent in agents:
                position = agent.position

                canvas_pos = self._utils.get_canvas_position(agent.position)
                agent.angle = self.angle[EDirection.Left.name]
                agent.img_ref = canvas.create_image(side, canvas_pos['x'], canvas_pos['y'],
                                                    center_origin=True, scale_type=ScaleType.ScaleToWidth,
                                                    scale_value=self._cell_size)
                self._img_refs[side][agent.id] = agent.img_ref


class GuiUtils:

    def __init__(self, cell_size):
        self._cell_size = cell_size

    def get_canvas_position(self, position, center_origin=True):
        addition = int(self._cell_size / 2) if center_origin else 0
        return {
            'x': position.x * self._cell_size + addition,
            'y': position.y * self._cell_size + addition
        }

    def _get_line_xys(self, agent, curr_val, max_val, offset):
        canvas_pos = self.get_canvas_position(agent.position.x, agent.position.y)
        y1 = y2 = canvas_pos['y'] + int(self._cell_size / 2) - 10 + offset
        x1 = canvas_pos['x'] - int(self._cell_size / 2) + 5
        if curr_val == 0:
            x2 = x1
        else:
            x2 = x1 + math.ceil((self._cell_size - 10) * (curr_val / max_val))

        return x1, y1, x2, y2
