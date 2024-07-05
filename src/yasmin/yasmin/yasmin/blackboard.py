# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from typing import Any


class Blackboard(object):
    def __init__(self, init=None) -> None:
        if init is not None:
            self.__dict__.update(init)

    def __getitem__(self, key) -> Any:
        return self.__dict__[key]

    def __setitem__(self, key, value) -> None:
        self.__dict__[key] = value

    def __delitem__(self, key) -> None:
        del self.__dict__[key]

    def __contains__(self, key) -> bool:
        return key in self.__dict__

    def __len__(self) -> int:
        return len(self.__dict__)

    def __repr__(self) -> str:
        return repr(self.__dict__)
