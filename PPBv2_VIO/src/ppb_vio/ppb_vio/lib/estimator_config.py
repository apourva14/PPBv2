"""Materialize an estimator config directory for one run (no ROS imports).

MINS and OpenVINS read OpenCV-FileStorage YAML ("%YAML:1.0"), which standard
YAML parsers reject, so overrides are applied as strict line edits: each
override must match exactly one `key:` line inside its section or the run is
refused. The upstream (GPL-3) configs are never copied into this repository;
they are read from the pinned estimator install and the edited copy is written
into the run's output directory, so every run records the exact config used.
"""

from dataclasses import dataclass
import json
from pathlib import Path
import re
import shutil


class ConfigError(RuntimeError):
    pass


@dataclass(frozen=True)
class Override:
    file: str           # file name inside the config directory
    section: str        # top-level key ('' for a top-level scalar)
    key: str
    value: object       # str, bool, int, float


def _format_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return repr(value)
    if isinstance(value, str):
        if '"' in value or '\n' in value:
            raise ConfigError(f'unsupported characters in string value {value!r}')
        return f'"{value}"'
    raise ConfigError(f'unsupported value type {type(value).__name__}')


def _section_bounds(lines, section):
    """Return [start, end) line indices of the block under top-level `section:`."""
    header = re.compile(rf'^{re.escape(section)}:\s*(#.*)?$')
    starts = [i for i, line in enumerate(lines) if header.match(line)]
    if len(starts) != 1:
        raise ConfigError(f'section {section!r} found {len(starts)} times (expected 1)')
    start = starts[0] + 1
    end = start
    while end < len(lines) and (not lines[end].strip() or lines[end][0] in ' \t#'):
        end += 1
    return start, end


def apply_override(text, section, key, value):
    """Return `text` with exactly one `key:` line in `section` replaced."""
    lines = text.split('\n')
    if section:
        start, end = _section_bounds(lines, section)
        pattern = re.compile(rf'^(\s+){re.escape(key)}:(\s*)([^#]*?)(\s*#.*)?$')
    else:
        start, end = 0, len(lines)
        pattern = re.compile(rf'^(){re.escape(key)}:(\s*)([^#]*?)(\s*#.*)?$')

    hits = [i for i in range(start, end) if pattern.match(lines[i])]
    # Within a section only direct children count (smallest indentation).
    if section and hits:
        indent = min(len(pattern.match(lines[i]).group(1)) for i in hits)
        hits = [i for i in hits if len(pattern.match(lines[i]).group(1)) == indent]
    where = f'{section}.{key}' if section else key
    if len(hits) != 1:
        raise ConfigError(f'{where}: matched {len(hits)} lines (expected exactly 1)')

    i = hits[0]
    m = pattern.match(lines[i])
    old = m.group(3)
    if old[:1] in '"\'' and (len(old) < 2 or old[-1] != old[0]):
        raise ConfigError(f'{where}: cannot safely parse existing value {lines[i]!r}')
    comment = m.group(4) or ''
    lines[i] = f'{m.group(1)}{key}: {_format_value(value)}{comment}'
    return '\n'.join(lines)


def materialize(src_dir, dst_dir, overrides, forbidden_files=()):
    """Copy `src_dir` to `dst_dir` and apply `overrides`.

    `forbidden_files` lists files whose presence would enable a sensor that
    this run must not use (MINS enables a sensor iff its config file exists).
    Returns the path of the written manifest.
    """
    src = Path(src_dir)
    dst = Path(dst_dir)
    if not src.is_dir():
        raise ConfigError(f'estimator config directory not found: {src}')
    if dst.exists():
        raise ConfigError(f'refusing to overwrite existing config directory: {dst}')

    present = sorted(name for name in forbidden_files if (src / name).exists())
    if present:
        raise ConfigError(f'{src} contains {present}, which would enable extra sensors')

    shutil.copytree(src, dst)
    for ov in overrides:
        path = dst / ov.file
        if not path.is_file():
            raise ConfigError(f'override target missing: {path}')
        text = path.read_text()
        try:
            path.write_text(apply_override(text, ov.section, ov.key, ov.value))
        except ConfigError as exc:
            raise ConfigError(f'{ov.file}: {exc}') from None

    manifest = dst / 'ppb_vio_manifest.json'
    manifest.write_text(json.dumps({
        'source': str(src.resolve()),
        'overrides': [ov.__dict__ for ov in overrides],
        'forbidden_files_checked': list(forbidden_files),
    }, indent=2))
    return manifest
