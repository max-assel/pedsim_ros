import typing
from ..human import Human

class Adult(Human):

    @classmethod
    def parse(cls, config: typing.Dict) -> typing.Dict:
        ...