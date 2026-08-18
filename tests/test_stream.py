"""Tests for streaming plugin."""
import asyncio

import logging
from tests.conftest import TCPStreamer
import dronemanager.core


class ImageChecker:
    """Test class which tracks how often its callback was called."""
    def __init__(self):
        """Create ImageChecker."""
        self.counter = 0  #: Counter for the callback

    def _img_callback(self, img: any):
        """Increment the counter everytime this function is called.

        Args:
            img: Unused callback argument.
        """
        self.counter += 1


async def test_stream(dm: dronemanager.core.DroneManager, video_stream_source: TCPStreamer):
    """Test the stream plugin with random images from a dummy source.

    Args:
        dm: DroneManager instance.
        video_stream_source: Pytest dummy server which sends random images.
    """
    stream_test_duration = 2
    checker = ImageChecker()

    # Load the plugin and start the stream for the test duration
    await dm.load_plugin("stream")
    stream = getattr(dm, "stream", None)
    assert stream is not None
    stream.add_callback(checker._img_callback)
    # await stream.display()  # Doesn't work, on headless runners.
    await stream.start_stream()
    await asyncio.sleep(stream_test_duration)

    # Try to start an already started stream: Should just return False
    res = await stream.start_stream()
    assert not res

    # Cleanup and check that we got approximately the expected number of images.
    stream.remove_callback(checker._img_callback)
    await stream.stop_stream()
    logging.error(f"{checker.counter}")
    assert abs(checker.counter - video_stream_source.frequency * stream_test_duration) <= 1
