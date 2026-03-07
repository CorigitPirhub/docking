from collections import Counter
from pathlib import Path
import json

from docking.dockbench import FAMILIES, DIFFICULTIES, load_manifest, load_representatives


DATASET_ROOT = Path("data/dockbench_v1")


def test_dockbench_manifest_is_balanced() -> None:
    manifest = load_manifest(DATASET_ROOT)
    assert len(manifest) == 72
    split_counts = Counter(spec.split for spec in manifest)
    assert split_counts == {"tuning": 12, "test": 48, "challenge": 12}
    family_counts = Counter(spec.family for spec in manifest)
    assert family_counts == {family: 18 for family in FAMILIES}
    difficulty_counts = Counter(spec.difficulty for spec in manifest)
    assert difficulty_counts == {difficulty: 24 for difficulty in DIFFICULTIES}
    cell_counts = Counter((spec.family, spec.difficulty) for spec in manifest)
    assert all(count == 6 for count in cell_counts.values())


def test_dockbench_representatives_cover_required_stage1_cases() -> None:
    reps = load_representatives(DATASET_ROOT)
    assert set(reps) == {"CF_L2", "SC_L2", "FC_L2", "EC_L2"}
    for key, spec in reps.items():
        assert spec.split == "test"
        assert spec.difficulty == "L2"
        assert Path(spec.scenario_json).exists()
        assert spec.family == key.split("_")[0]


def test_dockbench_scene_schema_contains_runtime_payload() -> None:
    manifest = load_manifest(DATASET_ROOT, split="test")
    scene = json.loads(Path(manifest[0].scenario_json).read_text(encoding="utf-8"))
    assert scene["schema_version"] == "dockbench-v1.0"
    assert scene["family"] in FAMILIES
    assert scene["difficulty"] in DIFFICULTIES
    assert "scenario" in scene
    assert scene["scenario"]["subset_tag"]
    assert all("role" in obstacle for obstacle in scene["obstacles"])
