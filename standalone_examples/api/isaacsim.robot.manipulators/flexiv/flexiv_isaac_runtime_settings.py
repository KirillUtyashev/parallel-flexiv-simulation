"""Shared Kit runtime settings for the retained Flexiv Isaac applications."""

from __future__ import annotations

import carb


SCENE_GRAPH_INSTANCING_SETTING = "/persistent/omnihydra/useSceneGraphInstancing"


def configure_scene_graph_instancing(enabled: bool) -> bool:
    """Set OmniHydra scene-graph instancing and verify the effective value."""

    if not isinstance(enabled, bool):
        raise TypeError("scene.scene_graph_instancing must be true or false")
    settings = carb.settings.get_settings()
    settings.set_bool(SCENE_GRAPH_INSTANCING_SETTING, enabled)
    effective = settings.get_as_bool(SCENE_GRAPH_INSTANCING_SETTING)
    if effective != enabled:
        raise RuntimeError(
            "could not apply OmniHydra scene-graph instancing: "
            f"requested={enabled} effective={effective}"
        )
    return effective
