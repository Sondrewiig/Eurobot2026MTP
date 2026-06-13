import os
import yaml
from ament_index_python.packages import get_package_share_directory


def default_tags_yaml() -> str:
    return os.path.join(
        get_package_share_directory("main_bot_control"),
        "config",
        "tags.yaml",
    )


def load_tag_registry(path: str | None = None) -> dict:
    if path is None:
        path = default_tags_yaml()

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    tags = data.get("tags", {})
    return {int(tag_id): info for tag_id, info in tags.items()}