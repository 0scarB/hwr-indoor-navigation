import functools
from typing import Any, cast, Protocol, Type, TypeVar

from hwr_indoor_navigation import error


class _AttributeSetterDeleter(Protocol):

    def __setattr__(self, name: str, value: Any) -> None: ...

    def __delattr__(self, name: str) -> None: ...


TAttributeSetterDeleter = TypeVar("TAttributeSetterDeleter", bound=_AttributeSetterDeleter)


def externally_immutable_cls(cls: Type[TAttributeSetterDeleter]) -> Type[TAttributeSetterDeleter]:

    class WrapperCls:
        _inst: TAttributeSetterDeleter

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            object.__setattr__(self, "_inst", cls(*args, **kwargs))

        def __getattr__(self, name: str) -> Any:
            return getattr(self._inst, name)

        def __setattr__(self, name: str, value: Any) -> None:
            raise errors.MutabilityDisallowed(
                f"Cannot set attribute on immutable class: `{type(self._inst).__name__}(...).{name} = {repr(value)}`"
            )

        def __delattr__(self, name: str) -> None:
            raise errors.MutabilityDisallowed(
                f"Cannot delete attribute on immutable class: `del {type(self._inst).__name__}(...).{name}`"
            )

    # Copy docstrings and annotations
    WrapperCls.__doc__ = cls.__doc__
    WrapperCls.__init__.__doc__ = cls.__init__.__doc__
    WrapperCls.__init__.__annotations__ = cls.__init__.__annotations__

    return cast(Type[TAttributeSetterDeleter], WrapperCls)
