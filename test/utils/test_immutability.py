import pytest

from dataclasses import dataclass

from hwr_indoor_navigation.utils.immutability import externally_immutable_cls
from hwr_indoor_navigation import errors


def test_can_access_attribute_on_externally_immutable_cls():
    expected_val = "should be accessible"

    @externally_immutable_cls
    class Cls:

        def __init__(self):
            self.val = expected_val

    inst = Cls()

    assert inst.val == expected_val


def test_can_call_method_on_externally_immutable_cls():

    @externally_immutable_cls
    class Sum:

        def __init__(self, left, right):
            self.left = left
            self.right = right

        def calculate(self):
            return self.left + self.right

    assert Sum(1, 2).calculate() == 3


def test_cannot_set_attribute_on_externally_immutable_cls():
    @externally_immutable_cls
    class Cls:

        def __init__(self):
            self.val = "can assign within class"

    inst = Cls()

    with pytest.raises(errors.MutabilityDisallowed):
        inst.val = "should fail"


def test_cannot_delete_attribute_on_externally_immutable_cls():
    @externally_immutable_cls
    class Cls:

        def __init__(self):
            self.val = "can assign within class"

    inst = Cls()

    with pytest.raises(errors.MutabilityDisallowed):
        del inst.val


def test_can_make_dataclass_externally_immutable():

    @externally_immutable_cls
    @dataclass
    class Data:
        a: int = 1
        b: int = 2

    assert Data().a == 1
    assert Data().b == 2

    with pytest.raises(errors.MutabilityDisallowed):
        Data().a = 3

    with pytest.raises(errors.MutabilityDisallowed):
        del Data().b


def test_can_make_inherited_dataclass_externally_immutable():

    @dataclass
    class Base:
        a: int = 1
        b: int = 2

    @externally_immutable_cls
    @dataclass
    class Data(Base):
        c: int = 3
        d: int = 4

    assert Data().a == 1
    assert Data().b == 2
    assert Data().c == 3
    assert Data().d == 4

    with pytest.raises(errors.MutabilityDisallowed):
        Data().a = 5

    with pytest.raises(errors.MutabilityDisallowed):
        Data().c = 6

    with pytest.raises(errors.MutabilityDisallowed):
        del Data().b

    with pytest.raises(errors.MutabilityDisallowed):
        del Data().d
