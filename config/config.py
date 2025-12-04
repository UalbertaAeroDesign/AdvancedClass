from typing import Literal
import yaml
import os
from threading import Lock
from dataclasses import dataclass

@dataclass
class ConfigData:
    """
    Configuration attributes.

    Attributes:
        camera_type: Type of camera. Can be "webcam", "feed", or "serial".
        feed_url: URL of the feed if camera_type is "feed".
        serial_device: Serial device name if camera_type is "serial".
    """
    camera_type: Literal["webcam", "feed", "serial"] = "webcam"
    feed_url: str = ""
    serial_device: str = ""


class Config:
    _instance = None
    _lock = Lock()

    # Statically declare attributes for IDE
    camera_type: str
    feed_url: str
    serial_device: str

    def __new__(cls, config_path: str = "config.yml"):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._init_config(config_path)
        return cls._instance

    def _init_config(self, config_path: str):
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r") as f:
            try:
                yaml_config = yaml.safe_load(f) or {}
            except yaml.YAMLError as e:
                raise ValueError(f"Error parsing config file: {e}")

        config_section = yaml_config.get("config", {})

        # Set attributes directly so IDE can see them
        self.camera_type: Literal["webcam", "feed", "serial"] = config_section.get("camera_type", "webcam")
        self.feed_url = config_section.get("feed_url", "")
        self.serial_device = config_section.get("serial_device", "")

    def __repr__(self):
        return (f"<Config camera_type={self.camera_type}, "
                f"feed_url={self.feed_url}, serial_device={self.serial_device}>")
