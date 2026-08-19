"""Sensor plugin for an ecowitt GW1100."""
import asyncio
import datetime
import logging
import math
import requests

import dronemanager.core
from dronemanager.plugins.sensor import Sensor

# Leitstand IP: 192.168.1.41
# Use HTTP GET with http://192.168.1.41/get_livedata_info,
# see https://blog.meteodrenthe.nl/2023/02/03/how-to-use-the-ecowitt-gateway-gw1000-gw1100-local-api/#


ECOWITT_ID_MAP_COMMON = {
    "0x02": "temperature",
    "0x03": "dew_point",
    "0x07": "humidity",
    "0x15": "light",
    "0x17": "uvi",
    "0x19": "wind_speed_max_day",
    "0x0A": "wind_direction",
    "0x0B": "wind_speed",
    "0x0C": "gust_speed",
}
"""Maps code bytes for "common" entries to understandable strings.

:meta hide-value:"""


ECOWITT_ID_MAP_RAIN = {
    "0x0D": "rain_event",
    "0x0E": "rain_rate",
    "0x10": "cum_rain_today",
    "0x11": "cum_rain_week",
    "0x12": "cum_rain_month",
    "0x13": "cum_rain_year",
}
"""Maps code bytes for "rain" entries to understandable strings.

:meta hide-value:"""


class WeatherDataEntry:
    """Data class for a single type of weather information.

    Contains components: The label for the information, the value and the unit, i.e. "temperature", 25.3, "C".
    """
    def __init__(self, name: str, value: float = math.nan, unit: str = ""):
        """Create a WeatherDataEntry object.

        Args:
            name: A label for the information.
            value: The value of the metric. Default ``math.nan``.
            unit: The unit of the metric. Default "".
        """
        self.name: str = name  #: A label for the information.
        self.value: float = value  #: The value of the metric.
        self.unit: str = unit  #: The unit of the metric.


class WeatherData:
    """Data class for weather information."""

    def __init__(self, timestamp: datetime.datetime):
        """Create WeatherData object.

        Args:
            timestamp: A time stamp for the data.
        """
        self.temperature: WeatherDataEntry = WeatherDataEntry("Temperature")  #: Current temperature
        self.dew_point: WeatherDataEntry = WeatherDataEntry("Dew point")  #: Current dew point
        self.humidity: WeatherDataEntry = WeatherDataEntry("Humidity")  #: Current humidity
        self.light: WeatherDataEntry = WeatherDataEntry("Light")  #: Current solar irradiance
        self.uvi: WeatherDataEntry = WeatherDataEntry("UVI")  #: Current UV index
        self.wind_speed: WeatherDataEntry = WeatherDataEntry("Wind Speed")  #: Current wind speed
        self.wind_direction: WeatherDataEntry = WeatherDataEntry("Wind Direction")  #: Wind direction, 0 North
        self.gust_speed: WeatherDataEntry = WeatherDataEntry("Gust Speed")  #: Gust speed
        self.wind_speed_max_day: WeatherDataEntry = WeatherDataEntry("Day max wind")  #: Highest wind speed today

        self.rain_event: WeatherDataEntry = WeatherDataEntry("Rain Event")  #: Rain event
        self.rain_rate: WeatherDataEntry = WeatherDataEntry("Current Rain Rate")  #: Current rain rate
        self.cum_rain_today: WeatherDataEntry = WeatherDataEntry("Rain this Day")  # Cumulative rain this day
        self.cum_rain_week: WeatherDataEntry = WeatherDataEntry("Rain this Week")  #: Cumulative rain this week
        self.cum_rain_month: WeatherDataEntry = WeatherDataEntry("Rain this Month")  #: Cumulative rain this month
        self.cum_rain_year: WeatherDataEntry = WeatherDataEntry("Rain Year")  #: Cumulative rain this year

        self.pressure: WeatherDataEntry = WeatherDataEntry("Pressure")  #: Atmospheric pressure
        self.time: datetime.datetime = timestamp  #: The time of the data collection
        #: A list of the attribute, other than time, that get written out
        self.data_entries: list[WeatherDataEntry] = [self.temperature, self.dew_point, self.humidity, self.light,
                                                     self.uvi, self.wind_speed, self.wind_direction, self.gust_speed,
                                                     self.wind_speed_max_day, self.rain_event, self.rain_rate,
                                                     self.cum_rain_today, self.cum_rain_week, self.cum_rain_month,
                                                     self.cum_rain_year, self.pressure]

    def __str__(self) -> str:
        """Return weather data as a nice string representation.

        Returns:
            A string with the weather data.
        """
        return f"Time {self.time}\t" + "\t".join([f"{entry.name}: {entry.value}{entry.unit}"
                                                  for entry in self.data_entries])

    @classmethod
    def from_response_json(cls, response_json: dict, timestamp: datetime.datetime) -> "WeatherData":
        """Parse the response json into a WeatherData object.

        Also translates the id bytes to readable strings.

        Args:
            response_json: The json to be parsed.
            timestamp: The timestamp for the data.

        Returns:
            A WeaatherData object.
        """
        output = cls(timestamp=timestamp)
        # Parse common list entry
        if "common_list" in response_json:
            for entry in response_json["common_list"]:
                entry_id, entry_value, entry_unit = output.parse_xml_entry(entry)
                if entry_id in ECOWITT_ID_MAP_COMMON:
                    attr_name = ECOWITT_ID_MAP_COMMON[entry_id]
                    data_entry = output.__getattribute__(attr_name)
                    data_entry.value = entry_value
                    data_entry.unit = entry_unit
        if "rain" in response_json:
            for entry in response_json["rain"]:
                entry_id, entry_value, entry_unit = output.parse_xml_entry(entry)
                if entry_id in ECOWITT_ID_MAP_RAIN:
                    attr_name = ECOWITT_ID_MAP_RAIN[entry_id]
                    data_entry = output.__getattribute__(attr_name)
                    data_entry.value = entry_value
                    data_entry.unit = entry_unit
        if "wh25" in response_json:
            entry = response_json["wh25"][0]
            valunit = entry["abs"]
            value, unit = valunit.split(" ")
            output.pressure.value = value
            output.pressure.unit = unit
        return output

    def parse_xml_entry(self, entry: dict) -> tuple[str, float, str]:
        """Parse a single entry from the HTML response.

        Args:
            entry: The entry to be parsed.

        Returns:
            The parsed id, value and unit from the entry.
        """
        entry_id = entry.get("id")
        entry_value = entry.get("val", math.nan)
        entry_unit = entry.get("unit", None)
        if entry_unit is None:  # Unit probably in value
            splits = entry_value.split(" ")
            if len(splits) == 1:  # Either no unit in value, or a percentage
                if str(entry_value).endswith("%"):
                    entry_value = entry_value[:-1]
                    entry_unit = "%"
                else:
                    entry_unit = ""
            elif len(splits) == 2:  # Second entry is probably the unit
                entry_unit = splits[1]
                entry_value = splits[0]
        entry_value = float(entry_value)
        return entry_id, entry_value, entry_unit

    def to_json_dict(self) -> dict:
        """Create a json serializable dictionary.

        The output can be used to write the data to file, for example with json.dump.

        Returns:
            The data as a serializable dictionary.
        """
        out_dict = self.__dict__.copy()
        out_dict["time"] = self.time.isoformat()
        out_dict.pop("data_entries")
        for attr_name, attr_value in out_dict.items():
            if isinstance(attr_value, WeatherDataEntry):
                out_dict[attr_name] = attr_value.__dict__
        return out_dict

    @classmethod
    def from_json_dict(cls, json_dict: dict) -> "WeatherData":
        """Recreate the object from a dictionary, such as from json.loads.

        Args:
            json_dict: The json dictionary.

        Returns:
            The data as a WeatherData object.
        """
        timestamp = datetime.datetime.fromisoformat(json_dict.pop("time"))
        new_obj = cls(timestamp)
        for attr_name, attr_value in json_dict.items():
            entry = new_obj.__getattribute__(attr_name)
            entry.name = attr_value["name"]
            entry.value = attr_value["value"]
            entry.unit = attr_value["unit"]
        return new_obj


class EcoWittSensor(Sensor):
    """Class for EcoWitt Weather stations that support their HTTP API."""

    DEPENDENCIES: list[str] = ["sensor"]
    """Sensor plugin must be loaded before this can be loaded."""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str = "ecowitt"):
        """Create an EcoWittSensor.

        Args:
            dm: The associated DroneManager instance.
            logger: The logger to use for any output or errors.
            name: The name to use for this plugin. Default "ecowitt".
        """
        super().__init__(dm, logger, name)
        self.ip: str = "192.168.1.41"  #: The ip of the sensor.
        self.last_data: WeatherData | None = None  #: The last data we received.

    async def connect(self, ip: str | None = None) -> bool:
        """Connect to the EcoWitt sensor.

        No connection procedure as such. Instead, we try to request the data and return false if there is no response
        or an exception.

        Args:
            ip: The IP address of the sensor. If None, use a default value.

        Returns:
            Whether we got a response with HTTP status code 200.
        """
        await super().connect(ip=ip)
        if ip is None:
            ip = self.ip
        self.logger.info(f"Adding sensor with {ip}...")
        try:
            response = await asyncio.get_running_loop().run_in_executor(None, _get_data, ip)
        except Exception as e:
            self.logger.warning(f"No response from Ecowitt sensor at {ip}")
            self.logger.debug(repr(e), exc_info=True)
            return False
        if response.status_code == 200:
            self.ip = ip
            self.logger.info(f"Connected to sensor at {self.ip}")
            return True
        else:
            self.logger.warning(f"Received a response code {response.status_code}")
            return False

    async def get_data(self) -> WeatherData | None:
        """Request data from the sensor.

        Gets an HTTP response from the sensor and then parses that into :py:class:`WeatherData`.
        The timestamp is applied by this function, not the sensor itself.

        Returns None if there was an error.

        Returns:
            Either the data or None if there was an error.
        """
        try:
            timestamp = datetime.datetime.now(datetime.timezone.utc)
            response = await asyncio.get_running_loop().run_in_executor(None, _get_data, self.ip)
            if response.status_code == 200:
                data = WeatherData.from_response_json(response.json(), timestamp=timestamp)
                self.last_data = data
                return data
            else:
                self.logger.warning(f"Received a non-OK response code {response.status_code} from sensor!")
                return None
        except Exception as e:
            self.logger.warning("Couldn't get a response from sensor!")
            self.logger.debug(repr(e), exc_info=True)
            return None

    async def status(self):
        """No status as such to report, instead print core information."""
        self.logger.info(f"EcoWitt sensor as {self.PREFIX} with IP {self.ip}")

    async def disconnect(self):
        """Dummy function since there is no disconnect procedure."""
        pass


def _get_data(ip: str) -> requests.Response:
    """Partial requests get with timeout.

    Args:
        ip: IP of the sensor.

    Returns:
        The result of the requests call.
    """
    return requests.get(f"http://{ip}/get_livedata_info", timeout=3)
