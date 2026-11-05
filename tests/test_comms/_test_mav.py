"""Tests for the MAVSDK comm protocol and related implementing classes."""
import asyncio
from unittest.mock import Mock

from mavsdk.asyncio.plugins.mocap import MocapError

from dronemanager.core import DroneManager
from dronemanager.plugins.NatNet.MoCapData import generate_mocap_data
