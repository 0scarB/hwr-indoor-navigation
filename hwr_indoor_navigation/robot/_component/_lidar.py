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
        self.is_capturing = False

    def capture_output(self) -> Generator["Lidar.Output", None, None]:
        with open("lidar_out.txt", "w") as fdw:
            with subprocess.Popen(
                [f"{os.path.dirname(__file__)}/capture-lidar-data-and-print-to-stdout"],
                encoding="utf-8",  # needed for text instead of bytes
                stdout=fdw
            ) as proc:
                with open("lidar_out.txt", "r") as fdr:
                    self.is_capturing = True
                    yield from self.read_lidar_data_from_file_descriptor(cast(IO[str], fdr))
                    print("test")
                    self.is_capturing = False

    def read_lidar_data_from_file_descriptor(self, fd: IO[str]) -> Generator["Lidar.Output", None, None]:
        for line_group in self.read_line_groups_from_file_descriptor(fd):
            yield self.Output(
                int(line_group[0]),
                float(line_group[1]),
                float(line_group[2])
            )

    def read_line_groups_from_file_descriptor(self, fd: IO[str]) -> Generator[List[str], None, None]:
        while self.is_capturing:
            line_group: List[str] = []
            while self.is_capturing:
                line = fd.readline().rstrip()
                if line == "":
                    if len(line_group) == 3:
                        yield line_group
                    line_group.clear()
                else:
                    line_group.append(line)