# -*- coding: utf-8 -*-

# python imports
from __future__ import division

# chillin imports
from chillin_server import RealtimeGameHandler

# project imports
from handlers import map_handler, logic_handler, gui_handler


class GameHandler(RealtimeGameHandler):

    def on_recv_command(self, side_name, agent_name, command_type, command):
        if None in command.__dict__.values():
            print("None in command: %s - %s" % (side_name, command_type))
            return

    def on_initialize(self):

        print('initialize')
        self._map_handler = map_handler.MapHandler(self.sides)
        self.world, char_board = self._map_handler.load_map(self.config)
        self._logic_handler = logic_handler.LogicHandler(self.world, self.sides, char_board)
        self._logic_handler.initialize(self.canvas, self.config)

    def on_initialize_gui(self):
        print('initialize gui')
        self._gui_handler = gui_handler.GuiHandler(self.world, self.sides)
        self._gui_handler.initialize(self.canvas, self.config, self.world)
        # Apply actions
        self.canvas.apply_actions()

    def on_process_cycle(self):
        print('cycle %i' % (self.current_cycle,))
        if self.world.validate_command(None, None):
            self.world.apply_command(None, None)

    def on_update_clients(self):
        print('update clients')
        self.send_snapshot(self.world)

    def on_update_gui(self):
        print('update gui')
        self.canvas.apply_actions()
