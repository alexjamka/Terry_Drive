"""
Face database utilities for Buddy.

Stores 512-d ArcFace embeddings per person under ~/buddy_ws/face_db/<name>/*.npy
Provides enrollment (save) and recognition (match) functions.
"""

from pathlib import Path
import json
import time
import numpy as np


FACE_DB_PATH = Path.home() / "buddy_ws" / "face_db"
RECOGNITION_THRESHOLD = 0.50


def ensure_db() -> None:
    FACE_DB_PATH.mkdir(parents=True, exist_ok=True)


def person_dir(name: str) -> Path:
    return FACE_DB_PATH / name.lower().strip()


def save_embedding(name: str, embedding: np.ndarray) -> Path:
    ensure_db()
    d = person_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    idx = len(list(d.glob("*.npy")))
    out = d / f"{idx:04d}.npy"
    np.save(out, embedding.astype(np.float32))
    _touch_meta(name)
    return out


def _touch_meta(name: str) -> None:
    meta_path = person_dir(name) / "meta.json"
    samples = len(list(person_dir(name).glob("*.npy")))
    data = {"name": name, "samples": samples, "last_updated": time.time()}
    if not meta_path.exists():
        data["enrolled_at"] = time.time()
    else:
        try:
            existing = json.loads(meta_path.read_text())
            data["enrolled_at"] = existing.get("enrolled_at", time.time())
        except Exception:
            data["enrolled_at"] = time.time()
    meta_path.write_text(json.dumps(data, indent=2))


def load_db() -> dict:
    ensure_db()
    db = {}
    for person in FACE_DB_PATH.iterdir():
        if not person.is_dir():
            continue
        embs = []
        for npy in sorted(person.glob("*.npy")):
            try:
                embs.append(np.load(npy))
            except Exception:
                continue
        if embs:
            db[person.name] = np.stack(embs, axis=0)
    return db


def _normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.clip(norm, 1e-8, None)


def match(embedding: np.ndarray, db: dict) -> tuple:
    if not db:
        return ("unknown", 0.0)
    q = _normalize(embedding.reshape(1, -1))
    best_name = "unknown"
    best_sim = -1.0
    for name, embs in db.items():
        refs = _normalize(embs)
        sims = (refs @ q.T).flatten()
        top = float(np.max(sims))
        if top > best_sim:
            best_sim = top
            best_name = name
    if best_sim < RECOGNITION_THRESHOLD:
        return ("unknown", best_sim)
    return (best_name, best_sim)


def list_enrolled() -> list:
    ensure_db()
    out = []
    for person in sorted(FACE_DB_PATH.iterdir()):
        if not person.is_dir():
            continue
        n = len(list(person.glob("*.npy")))
        if n > 0:
            out.append((person.name, n))
    return out
