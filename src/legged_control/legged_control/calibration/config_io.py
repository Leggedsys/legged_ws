"""ruamel.yaml-based config reader/writer that preserves comments."""

from __future__ import annotations
import pathlib
from ruamel.yaml import YAML


class ConfigIO:
    def __init__(self, path: str | pathlib.Path) -> None:
        self._path = pathlib.Path(path)
        self._yaml = YAML()
        self._yaml.preserve_quotes = True

    def read(self) -> dict:
        with open(self._path) as f:
            return self._yaml.load(f)

    def patch(self, updates: dict) -> None:
        """Deep-merge `updates` into the yaml file, preserving comments."""
        data = self.read()
        _deep_merge(data, updates)
        with open(self._path, 'w') as f:
            self._yaml.dump(data, f)

    def patch_joint(self, joint_name: str, field: str, value) -> None:
        """Update a single field on the joint entry matching `joint_name`."""
        data = self.read()
        for entry in data.get('joints', []):
            if entry.get('name') == joint_name:
                entry[field] = value
                break
        with open(self._path, 'w') as f:
            self._yaml.dump(data, f)


def _deep_merge(base: dict, updates: dict) -> None:
    for k, v in updates.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
