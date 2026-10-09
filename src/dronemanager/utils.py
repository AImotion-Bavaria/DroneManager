"""This module contains generic utility functions used throughout the software.

They mostly relate to GPS and NED positions.
Unless stated otherwise, a GPS coordinate is any indexable sequence with the latitude, longitude and AMSL in that order.
"""
import asyncio
from collections.abc import Sequence, MutableSet
from concurrent.futures import Future
from haversine import inverse_haversine, haversine, Direction, Unit
from importlib.resources import files
import logging
import math
import numpy as np
import pathlib
from platformdirs import user_documents_path
import socket
from urllib.parse import urlparse


COMMON_FORMATTER = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")
"""The common formatter string for the loggers.

:meta hide-value:
"""

EARTH_RADIUS = 6371000
"""Used to compute an approximate NED distance between two GPS coordinates.
"""

NAME = "DroneManager"

DOC_DIR = user_documents_path().joinpath(NAME)
"""The directory where DroneManager directories for drop-in files will be installed.

Defaults to the user "documents" folder.

:meta hide-value:
"""

SRC_DIR = pathlib.Path(__file__).parent
"""The source directory for DroneManager.

:meta hide-value:
"""

LOG_DIR = DOC_DIR.joinpath("Logs")
""" The directory where all the log files are saved.

:meta hide-value:
"""
LOG_DIR.mkdir(parents=True, exist_ok=True)

CACHE_DIR = DOC_DIR.joinpath(".cache")
""" The directory for any information that might be worth caching.

Currently only used for camera definition information.

:meta hide-value:
"""
CACHE_DIR.mkdir(parents=True, exist_ok=True)

_CONFIG_FILE = DOC_DIR.joinpath("config.json")
""" Location of the configuration file.

:meta hide-value:
"""


def get_config() -> pathlib.Path:
    """Returns the location of the configuration json.

    By template config is shipped with DroneManager, which is installed if the file is otherwise missing.

    Returns:
        The path to the config file.
    """
    if not _CONFIG_FILE.exists():
        default_config = files("dronemanager").joinpath(
            "resources/config.json"
        )

        _CONFIG_FILE.write_text(
            default_config.read_text(),
            encoding="utf-8",
        )

    return _CONFIG_FILE


def dist_ned(pos1: np.ndarray, pos2: np.ndarray) -> float:
    """The euclidian distance between two cartesian points.

    For n-dimensional points, the arrays should have shape (n,).

    Args:
        pos1: First position, as a numpy array
        pos2: Second position, as a numpy array

    Returns:
        The distance between the two points as a float.
    """
    return np.sqrt(np.sum((pos1 - pos2) ** 2, axis=0))


def dist_gps(gps1: Sequence[float], gps2: Sequence[float]) -> float:
    """Approximate euclidian distance between two GPS coordinates.

    Haversine functions with WGS84 are used to compute the horizontal component of the distance.

    Args:
        gps1: GPS coordinates of first point
        gps2: GPS coordinates of second point

    Returns:
        Approximate distance between the two points as a float.
    """
    dist_horiz = haversine((gps1[0], gps1[1]), (gps2[0], gps2[1]), unit=Unit.METERS)
    dist_alt = gps1[2] - gps2[2]
    return math.sqrt(dist_horiz * dist_horiz + dist_alt * dist_alt)


def heading_ned(pos1: Sequence[float], pos2: Sequence[float]) -> float:
    """The heading from one position to another, in degrees.

    This is the angle from north, such that moving in that direction from the first point will cross the second point.
    The range is from -180 to +180 with 0 north and +90 east. The two positions must be given as NED.
    Assumes flat earth.

    Args:
        pos1: The first position
        pos2: The second position

    Returns:
        The heading in degrees, as a float.
    """
    return math.atan2(pos2[1] - pos1[1], pos2[0] - pos1[0]) / math.pi * 180


def heading_gps(gps1: Sequence[float], gps2: Sequence[float]) -> float:
    """The heading from one GPS position to another, in degrees.

    This is the angle from north, such that moving in that direction from the first point will cross the second point.
    The range is from -180 to +180 with 0 north and +90 east. The two positions must be given as GPS coordinates with
    latitude first and longitude second.

    Args:
        gps1: The first position
        gps2: The second position

    Returns:
        The heading in degrees, as a float.
    """
    diff_long_rad = (gps2[1] - gps1[1]) * math.pi / 180
    lat1_rad = gps1[0] * math.pi / 180
    lat2_rad = gps2[0] * math.pi / 180
    x = math.sin(diff_long_rad) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_long_rad)
    return math.atan2(x, y) * 180 / math.pi


def relative_gps(gps: Sequence[float], offset: Sequence[float]) -> tuple[float, float, float]:
    """Create a GPS coordinate that is shifted from the input GPS by a NED input offset.

    Uses haversine functions.

    Args:
        gps: Initial GPS coordinate
        offset: NED offset

    Returns:
        The new, offset GPS coordinate as a tuple (latitude, longitude, amsl)
    """
    target_alt = gps[2] - offset[2]
    coords = (gps[0], gps[1])
    coords = inverse_haversine(coords, offset[0], Direction.NORTH, unit=Unit.METERS)
    target_lat, target_long = inverse_haversine(coords, offset[1], Direction.EAST, unit=Unit.METERS)
    return target_lat, target_long, target_alt


def offset_from_gps(origin: Sequence[float], gps1: Sequence[float], gps2: Sequence[float]) \
        -> tuple[float, float, float]:
    """Create GPS point offset from origin by same distance and heading as two other GPS points.

    Given two GPS points, computes the heading and distance between them and then creates a new point
    separated fom a third GPS point by the same heading and distance.
    Uses haversine functions.

    Args:
        origin: The GPS point from which the new offset point will be created
        gps1: The starting GPS point for determining the offset
        gps2: The ending GPS point for determining the offset

    Returns:
        The new, offset point as a tuple (latitude, longitude, amsl)
    """
    dist_horiz = haversine((gps1[0], gps1[1]), (gps2[0], gps2[1]), unit=Unit.METERS)
    dist_alt = gps2[2] - gps1[2]
    heading = heading_gps(gps1, gps2) * math.pi / 180
    lat, long = inverse_haversine(origin[:2], dist_horiz, heading, unit=Unit.METERS)
    return lat, long, origin[2] + dist_alt


def ned_from_gps(gps1: Sequence[float], gps2: Sequence[float]) -> tuple[float, float, float]:
    """Given two GPS points, compute the NED difference between the two positions.

    Utilizes the same algorithm as PX4 http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html

    Args:
        gps1: GPS coordinates of first point
        gps2: GPS coordinates of second point

    Returns:
        A tuple with the north, east, down offset between the two GPS coordinates.
    """
    lat1_rad = gps1[0] * math.pi / 180
    lat2_rad = gps2[0] * math.pi / 180
    long1_rad = gps1[1] * math.pi / 180
    long2_rad = gps2[1] * math.pi / 180
    cos_long_diff = math.cos(long2_rad - long1_rad)

    arg = math.sin(lat1_rad) * math.sin(lat2_rad) + math.cos(lat1_rad) * math.cos(lat2_rad) * cos_long_diff
    if arg > 1.0:
        arg = 1.0
    if arg < -1.0:
        arg = -1.0
    c = math.acos(arg)
    k = 1.0
    if abs(c) > 0:
        k = c / math.sin(c)

    north = k * (math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * cos_long_diff)
    east = k * math.cos(lat2_rad) * math.sin(long2_rad - long1_rad)
    down = gps2[2] - gps1[2]
    return north * EARTH_RADIUS, east * EARTH_RADIUS, down


def get_free_port() -> int:
    """Get a free network port.

    The port is not guaranteed to be free once the function returns, but they usually are.

    Returns:
        The free port
    """
    sock = socket.socket()
    sock.bind(("", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def parse_address(string: str) -> tuple[str, str, int]:
    """Parses a connection string of the form ``schema://host:appendix``.

    Also used to ensure that udp://:14540, udp://localhost:14540 and udp://127.0.0.1:14540 are recognized as equivalent.

    Missing elements from the string or the other entries are replaced with defaults.
    These are "udp", an empty host "", and 50051 for the scheme, host and appendix, respectively.

    Args:
        string: A connection string.

    Returns:
        A tuple with the parsed schema, host and appendix.
    """
    scheme, rest = string.split("://")
    if scheme == "serial":
        loc, append = rest.split(":")
        append = int(append)
    else:
        parse_drone_addr = urlparse(string)
        scheme = parse_drone_addr.scheme
        loc = parse_drone_addr.hostname
        append = parse_drone_addr.port
        if scheme is None:
            scheme = "udp"
        if loc is None:
            loc = ""
        if loc == "localhost":
            loc = ""
        elif loc == "127.0.0.1":
            loc = ""
        if append is None:
            append = 50051
    return scheme, loc, append


async def coroutine_awaiter(task: asyncio.Future | Future, logger: logging.Logger):
    """Awaits the provided coroutine, logging any exceptions.

    In DroneManager, there are many functions that run indefinitely without a concrete return value. These functions
    are not "naturally" awaited at any point in the code, which can lead to exceptions being silently dropped.
    This function is used to catch these scenarios, by creating an extra task that awaits the actually interesting task.
    This task does not need to be explicitly awaited itself.

    Args:
        task: The task or future to await.
        logger: This logger will be used for any exception logging.
    """
    try:
        if isinstance(task, asyncio.Future):
            await task
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error("Encountered an exception in a coroutine! See the log for more details")
        logger.debug(repr(e), exc_info=True)


def cancel_running_tasks(running_tasks: MutableSet[asyncio.Future | Future]):
    """Cancel all tasks in a collection of tasks.

    This is a common pattern in DroneManager, which is why the functionality is bundled here.

    Args:
        running_tasks: A set of tasks to be cancelled.
    """
    while len(running_tasks) > 0:
        task = running_tasks.pop()
        if isinstance(task, asyncio.Future) or isinstance(task, Future):
            task.cancel()


# The first haversine/ inverse_haversine calls are slow, because of numba stuff, so do it here
dist_gps([10, 20, 300], [20, 10, 400])
relative_gps([1, 1, 1], [1, 1, 1])
