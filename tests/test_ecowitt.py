"""Tests for the EcoWitt sensor plugin."""
from dronemanager.core import DroneManager
from dronemanager.sensors.ecowitt import WeatherData
import json

import responses
import logging


EXAMPLE_JSON = json.loads('{ "common_list": ['
                          '{ "id": "0x02", "val": "22.9", "unit": "C" },'
                          '{ "id": "0x07", "val": "41%" },'
                          '{ "id": "3", "val": "22.9", "unit": "C" },'
                          '{ "id": "0x03", "val": "9.0", "unit": "C" },'
                          '{ "id": "0x0B", "val": "0.4 m/s" },'
                          '{ "id": "0x0C", "val": "0.5 m/s" },'
                          '{ "id": "0x19", "val": "1.0 m/s" },'
                          '{ "id": "0x15", "val": "4.02 W/m2" },'
                          '{ "id": "0x17", "val": "0" },'
                          '{ "id": "0x0A", "val": "260" }],'
                          '"rain": ['
                          '{ "id": "0x0D", "val": "0.0 mm" },'
                          '{ "id": "0x0E", "val": "0.0 mm/Hr" },'
                          '{ "id": "0x10", "val": "0.0 mm" },'
                          '{ "id": "0x11", "val": "0.0 mm" },'
                          '{ "id": "0x12", "val": "0.0 mm" },'
                          '{ "id": "0x13", "val": "0.0 mm",'
                          '"battery": "0" }],'
                          '"wh25": ['
                          '{ "intemp": "22.9", "unit": "C", "inhumi": "42%", "abs": "975.8 hPa", "rel": "975.8 hPa" }]'
                          '}')
"""Example json for testing.

:meta hide-value:"""


@responses.activate
async def test_plugin_dummy(dm: DroneManager):
    """Test ecowitt plugin.

    Args:
        dm: DroneManager instance.
    """
    ip = "192.168.1.41"
    responses.add(
        responses.GET,
        f"http://{ip}/get_livedata_info",
        json=EXAMPLE_JSON,
        status=200,
    )
    await dm.load_plugin("sensor")
    sensor = await dm.sensor.load("ecowitt")
    sensor.ip = ip
    res = await dm.ecowitt.connect(ip)

    assert res

    responses.add(
        responses.GET,
        f"http://{ip}/get_livedata_info",
        json=EXAMPLE_JSON,
        status=200,
    )
    weather_data = await dm.ecowitt.get_data()
    logging.error(weather_data)
    json_dict = weather_data.to_json_dict()
    wd = WeatherData.from_json_dict(json_dict)

    assert wd.temperature.value == 22.9, "Failed to parse, save and reload weather data"

    await dm.ecowitt.status()

    responses.add(
        responses.GET,
        f"http://{ip}/get_livedata_info",
        json=EXAMPLE_JSON,
        status=404,
    )
    weather_data = await dm.ecowitt.get_data()
    assert weather_data is None

    responses.add(
        responses.GET,
        f"http://{ip}/get_livedata_info",
        json=EXAMPLE_JSON,
        status=200,
    )

    await dm.ecowitt.reconnect()
    await dm.ecowitt.disconnect()
