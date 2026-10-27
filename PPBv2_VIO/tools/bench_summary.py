#!/opt/vio-tools/bin/python -I
"""Collect VIO bench runs into one comparison table (Markdown + CSV).

    bench_summary.py /data/runs/*_bench_* --out /data/bench/summary

One row per run folder (run_info.yaml + evaluation.json, + estimator/serial_summary.json
and the input bag's .stress.json when present). Columns are what the frontend
selection is decided on (spec, "VIO frontend selection"): robustness (initialized,
coverage, divergence, health gate), accuracy (ATE, RTE), consistency (NEES), and
real-time factor on this machine.
"""

import argparse
import csv
import json
from pathlib import Path
import sys

import yaml

COLUMNS = ['run', 'frontend', 'sequence', 'profile', 'stress', 'result', 'ate_m', 'healthy_ate_m',
           'rte_m_per_10m', 'nees_pos', 'healthy_nees_pos', 'init_s', 'coverage', 'diverged_at_s', 'gate_at_s',
           'realtime_x', 'p95_update_ms']


def fmt(v, nd=3):
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def stress_label(bag, run_dir=None):
    # The suite deletes stressed bags after scoring and keeps their record in the run.
    report = run_dir / 'stress.json' if run_dir and (run_dir / 'stress.json').is_file() \
        else Path(str(bag) + '.stress.json')
    if not report.is_file():
        return 'none'
    r = json.loads(report.read_text())
    parts = [f'drop {r["drop_frames"]:.0%}' if r['drop_frames'] else '',
             f'blackout {r["blackout_s"][0]:g}-{r["blackout_s"][1]:g}s' if r['blackout_s'] else '',
             f'jitter {r["jitter_ms"]:g}ms' if r['jitter_ms'] else '',
             f'imu-drop {r["imu_drop"]:.0%}' if r['imu_drop'] else '',
             f'start {r["start_s"]:g}s' if r['start_s'] else '']
    return ', '.join(p for p in parts if p) or 'none'


def row(run_dir):
    info = yaml.safe_load((run_dir / 'run_info.yaml').read_text())
    ev_path = run_dir / 'evaluation.json'
    serial = run_dir / 'estimator' / 'serial_summary.json'
    rt = json.loads(serial.read_text())['realtime_factor'] if serial.is_file() else None
    base = {'run': run_dir.name, 'frontend': info.get('frontend', info.get('estimator')),
            'sequence': info.get('sequence'),
            'profile': Path(str(info.get('profile', '-'))).name.removesuffix('.yaml'),
            'stress': stress_label(info.get('input_bag', ''), run_dir), 'realtime_x': rt}
    if not ev_path.is_file():
        return {**base, 'result': 'NO OUTPUT'}
    ev = json.loads(ev_path.read_text())
    hg, nees = ev.get('health_gate', {}), ev.get('nees_position', {})
    result = ('INVALID' if not ev.get('valid', True) else
              'PASS' if ev.get('passed') else 'FAIL' if ev.get('passed') is False else 'REPORTED')
    return {**base, 'result': result, 'ate_m': ev['ape_rmse_m'],
            'healthy_ate_m': hg.get('healthy_ape_rmse_m'),
            'rte_m_per_10m': ev.get('rte_translation_rmse_m_per_10m'),
            'nees_pos': nees.get('mean'), 'healthy_nees_pos': hg.get('healthy_nees_position_mean'),
            'init_s': ev['availability']['init_latency_s'],
            'coverage': ev['availability']['coverage'],
            'diverged_at_s': hg.get('divergence_onset_s'), 'gate_at_s': hg.get('gate_error_s'),
            'p95_update_ms': (ev.get('compute') or {}).get('p95_ms')}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('runs', nargs='+', help='run folders')
    parser.add_argument('--out', required=True, help='output path prefix (.md and .csv are added)')
    args = parser.parse_args()
    rows = [row(Path(r)) for r in sorted(args.runs) if (Path(r) / 'run_info.yaml').is_file()]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out.with_suffix('.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    lines = ['| ' + ' | '.join(COLUMNS[1:]) + ' |', '|' + '---|' * (len(COLUMNS) - 1)]
    for r in rows:
        lines.append('| ' + ' | '.join(fmt(r.get(c), 2 if c in ('init_s', 'realtime_x') else 3)
                                       for c in COLUMNS[1:]) + ' |')
    out.with_suffix('.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
