#!/usr/bin/env python3
from dataclasses import dataclass
import subprocess
from typing import cast, Generator, IO, List
import os


class Lidar:

    @dataclass
    class Output:
        signal: int
        dist: float
        angle: float

    def __init__(self):
        pass

    def capture_output(self) -> Generator["Lidar.Output", None, None]:
        with subprocess.Popen(
            [f"{os.path.dirname(__file__)}/capture-lidar-data-and-print-to-stdout"],
            stdout=subprocess.PIPE,
            encoding="utf-8",  # needed for text instead of bytes
        ) as proc:
            yield from self.read_lidar_data_from_file_descriptor(cast(IO[str], proc.stdout))

    def read_lidar_data_from_file_descriptor(self, fd: IO[str]) -> Generator["Lidar.Output", None, None]:
        for line_group in self.read_line_groups_from_file_descriptor(fd):
            yield self.Output(
                int(line_group[0]),
                float(line_group[1]),
                float(line_group[2])
            )

    def read_line_groups_from_file_descriptor(self, fd: IO[str]) -> Generator[List[str], None, None]:
        while True:
            line_group: List[str] = []
            while True:
                line = fd.readline().rstrip()
                if line == "":
                    if len(line_group) == 3:
                        yield line_group
                    line_group.clear()
                else:
                    line_group.append(line)


if __name__ == "__main__":
    lidar = Lidar()
    for output in lidar.capture_output():
        print(output)