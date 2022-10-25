from interface import WithStartup, WithShutdown


class Component(WithStartup, WithShutdown):
    pass
