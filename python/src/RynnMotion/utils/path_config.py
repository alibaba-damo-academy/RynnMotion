"""
RynnMotion Path Configuration Module

This module provides utilities to extract custom path configurations from pyproject.toml
and resolve them relative to the project root. Works across different operating systems
and team members without hardcoded absolute paths.
"""

import os
import sys
from pathlib import Path
from typing import Dict, Optional, Any
import tomllib
import yaml



class PathConfig:
    """Manages path configurations from pyproject.toml"""

    def __init__(self, pyproject_path: Optional[Path] = None):
        """
        Initialize PathConfig

        Args:
            pyproject_path: Optional path to pyproject.toml. If None, searches from current file.
        """
        self._project_root = self._find_project_root(pyproject_path)
        self._config = self._load_config()
        self._paths = self._resolve_paths()

    def _find_project_root(self, pyproject_path: Optional[Path] = None) -> Path:
        """
        Find the project root directory containing pyproject.toml

        Args:
            pyproject_path: Optional explicit path to pyproject.toml

        Returns:
            Path to project root directory
        """
        if pyproject_path:
            if pyproject_path.is_file():
                return pyproject_path.parent
            elif pyproject_path.is_dir():
                pyproject_file = pyproject_path / "pyproject.toml"
                if pyproject_file.exists():
                    return pyproject_path

        # Search from current file location upward
        current = Path(__file__).resolve().parent

        while current != current.parent:  # Stop at filesystem root
            pyproject_file = current / "pyproject.toml"
            if pyproject_file.exists():
                return current
            current = current.parent

        # Fallback: search from current working directory
        current = Path.cwd()
        while current != current.parent:
            pyproject_file = current / "pyproject.toml"
            if pyproject_file.exists():
                return current
            current = current.parent

        raise FileNotFoundError(
            "Could not find pyproject.toml in current directory or any parent directories"
        )

    def _load_config(self) -> Dict:
        """Load configuration from pyproject.toml"""
        pyproject_file = self._project_root / "pyproject.toml"

        try:
            with open(pyproject_file, 'rb') as f:
                return tomllib.load(f)
        except Exception as e:
            raise RuntimeError(f"Failed to load {pyproject_file}: {e}")

    def _resolve_paths(self) -> Dict[str, Path]:
        """
        Resolve relative paths from pyproject.toml to absolute paths

        Returns:
            Dictionary mapping path names to resolved Path objects
        """
        tool_config = self._config.get('tool', {})
        rynnmotion_config = tool_config.get('rynnmotion', {})
        paths_config = rynnmotion_config.get('paths', {})

        resolved_paths = {}

        # Get the python directory (where this script is typically located)
        python_dir = self._project_root / "python"

        for path_name, relative_path in paths_config.items():
            if isinstance(relative_path, str):
                # Resolve relative to python/ directory as specified in your config
                if python_dir.exists():
                    resolved_path = (python_dir / relative_path).resolve()
                else:
                    # Fallback to project root if python/ doesn't exist
                    resolved_path = (self._project_root / relative_path).resolve()

                resolved_paths[path_name] = resolved_path

        return resolved_paths

    def get_path(self, path_name: str) -> Path:
        """
        Get resolved path by name

        Args:
            path_name: Name of the path from pyproject.toml (e.g., 'models_root')

        Returns:
            Resolved Path object

        Raises:
            KeyError: If path_name is not found in configuration
        """
        if path_name not in self._paths:
            available_paths = list(self._paths.keys())
            raise KeyError(
                f"Path '{path_name}' not found. Available paths: {available_paths}"
            )

        return self._paths[path_name]

    def get_all_paths(self) -> Dict[str, Path]:
        """Get all resolved paths as a dictionary"""
        return self._paths.copy()

    @property
    def project_root(self) -> Path:
        """Get the project root directory"""
        return self._project_root

    @property
    def models_root(self) -> Path:
        """Convenience property for models_root path"""
        return self.get_path('models_root')

    @property
    def config_root(self) -> Path:
        """Convenience property for config_root path"""
        return self.get_path('config_root')

    @property
    def rynnmotion_root(self) -> Path:
        """Convenience property for RynnMotion project root path"""
        return self.get_path('project_root')

    def ensure_paths_exist(self, create_missing: bool = False) -> Dict[str, bool]:
        """
        Check if all configured paths exist

        Args:
            create_missing: If True, create missing directories

        Returns:
            Dictionary mapping path names to existence status
        """
        status = {}

        for path_name, path in self._paths.items():
            exists = path.exists()
            status[path_name] = exists

            if not exists and create_missing:
                try:
                    path.mkdir(parents=True, exist_ok=True)
                    status[path_name] = True
                    print(f"Created directory: {path}")
                except Exception as e:
                    print(f"Failed to create directory {path}: {e}")

        return status


# Global instance for easy access
_global_config = None


def get_path_config() -> PathConfig:
    """Get global PathConfig instance (singleton pattern)"""
    global _global_config
    if _global_config is None:
        _global_config = PathConfig()
    return _global_config


def get_path(path_name: str) -> Path:
    """Convenience function to get a path by name"""
    return get_path_config().get_path(path_name)


def get_models_root() -> Path:
    """Convenience function to get models root path"""
    return get_path_config().models_root


def get_config_root() -> Path:
    """Convenience function to get config root path"""
    return get_path_config().config_root


def get_rynnmotion_root() -> Path:
    """Convenience function to get RynnMotion project root path"""
    return get_path_config().rynnmotion_root


def load_yaml_config(relative_path: str, from_root: str | Path = "config") -> Dict[str, Any]:
    """
    Load YAML configuration file.

    Args:
        relative_path: Path relative to the specified root
        from_root: Which root to use - can be:
            - String: "config", "models", or "project"
            - Path object: actual base path to use

    Returns:
        Loaded YAML as dictionary

    Raises:
        ValueError: If from_root string is not a valid option
        FileNotFoundError: If YAML file does not exist
        yaml.YAMLError: If YAML parsing fails

    Example:
        >>> config = load_yaml_config("robot.yaml")  # Loads from config_root
        >>> model_cfg = load_yaml_config("scene.yaml", from_root="models")
        >>> custom_cfg = load_yaml_config("robot.yaml", from_root=Path("/custom/path"))
    """
    path_config = get_path_config()

    # Handle both string and Path types for from_root
    if isinstance(from_root, (str, Path)):
        if isinstance(from_root, str):
            if from_root == "config":
                base_path = path_config.config_root
            elif from_root == "models":
                base_path = path_config.models_root
            elif from_root == "project":
                base_path = path_config.project_root
            else:
                raise ValueError(
                    f"Unknown root: {from_root}. Valid options: 'config', 'models', 'project'"
                )
        else:
            # from_root is already a Path object
            base_path = from_root
    else:
        raise ValueError(
            f"from_root must be a string or Path, got {type(from_root)}"
        )

    # Clean up relative path prefixes
    if relative_path.startswith("config/"):
        relative_path = relative_path[7:]
    elif relative_path.startswith("models/"):
        relative_path = relative_path[7:]

    yaml_path = base_path / relative_path

    if not yaml_path.exists():
        raise FileNotFoundError(f"YAML file not found: {yaml_path}")

    try:
        with open(yaml_path, "r") as f:
            return yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise yaml.YAMLError(f"Failed to parse YAML file {yaml_path}: {e}")


# Example usage and testing
if __name__ == "__main__":
    try:
        config = PathConfig()

        print(f"Project root: {config.project_root}")
        print(f"All configured paths:")

        for name, path in config.get_all_paths().items():
            exists = "✓" if path.exists() else "✗"
            print(f"  {name}: {path} [{exists}]")

        # Test convenience functions
        print(f"\nUsing convenience functions:")
        print(f"Models root: {get_models_root()}")
        print(f"Config root: {get_config_root()}")

        # Check path existence
        print(f"\nPath existence check:")
        status = config.ensure_paths_exist(create_missing=False)
        for name, exists in status.items():
            print(f"  {name}: {'exists' if exists else 'missing'}")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
