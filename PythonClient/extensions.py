# -*- coding: utf-8 -*-

# project imports
from ks.models import Position
from ks.commands import ECommandDirection

# Position
def _position_is_equal(self, position):

    return self.y == position.y and self.x == position.x
def _position_get_neighbours(self, world):
    neighbours = [Position(x=self.x - 1, y=self.y), Position(x=self.x + 1, y=self.y),
                  Position(x=self.x, y=self.y - 1), Position(x=self.x, y=self.y + 1)]
    valid_neighbours = []
    for neighbour in neighbours:
        if 0 <= neighbour.x < world.width and 0 <= neighbour.y < world.height:
            valid_neighbours.append(neighbour)
    return valid_neighbours
def _position_is_neighbour(self, position, world):

    return any(self == neighbour for neighbour in position.get_neighbours(world))
def _position_to_tuple(self):

    return (self.y, self.x)
def _position_from_tuple(t):

    return Position(x=t[0], y=t[1])
def _position_direction_to(self, next_pos):
    delta_x = next_pos.x - self.x
    delta_y = next_pos.y - self.y
    if delta_x == 0 and delta_y == 0:
        return None

    if delta_x == 0:
        return ECommandDirection.Up if delta_y < 0 else ECommandDirection.Down
    if delta_y == 0:
        return ECommandDirection.Left if delta_x < 0 else ECommandDirection.Right
def _position_repr(self):

    return 'Position: {}, {}'.format(self.y, self.x)

Position.__eq__ = _position_is_equal
Position.get_neighbours = _position_get_neighbours
Position.is_neighbour = _position_is_neighbour
Position.to_tuple = _position_to_tuple
Position.from_tuple = _position_from_tuple
Position.direction_to = _position_direction_to
Position.__repr__ = _position_repr
# End Position
