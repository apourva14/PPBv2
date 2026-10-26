#!/opt/vio-tools/bin/python -I
"""Compare estimated trajectories with ground truth, one column per dataset.

Each column shows the top view (estimates position+yaw aligned exactly as
evaluate_run.py aligns them, at the ground-truth point), the position error
against distance travelled (where drift shows), and a metrics table. Use it to
put new frontends / architectures next to the existing baselines.

    plot_trajectories.py --out /data/plots/compare.png \
        --panel /data/euroc/V1_01_easy/dataset.yaml \
                "OpenVINS=/data/runs/<run>/estimate.tum" \
        --panel /data/rosario_v1/sequence02/dataset.yaml \
                "Wheel odometry=/data/runs/<run>/estimate.tum"

estimate.tum is written by evaluate_run.py into every --run-dir it scores.
At most three estimates per dataset (more stop being distinguishable by color).
"""

import argparse
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_run import align_to_ground_truth, load_dataset, RTE_DELTA_M  # noqa: E402
from ppb_vio_tools.metrics import relative_translation_errors  # noqa: E402
from ppb_vio_tools.trajectory import read_tum  # noqa: E402

# Reference data-viz palette, light mode: categorical slots 1-3 (validated as a
# set for all-pairs use), ground truth in primary ink, recessive neutrals.
SERIES = ('#2a78d6', '#eb6834', '#1baf7a')
SURFACE, INK, INK_2, MUTED = '#fcfcfb', '#0b0b0b', '#52514e', '#898781'
GRID, AXIS = '#e1e0d9', '#c3c2b7'
LINE_PT = 1.5
MAX_SERIES = len(SERIES)


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.75)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelcolor=MUTED, labelsize=8)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)


def panel_data(dataset_yaml, series):
    ds = load_dataset(argparse.Namespace(dataset=dataset_yaml, gt=None, sensor_bag=None,
                                         sequence=None))
    meta = yaml.safe_load(Path(dataset_yaml).read_text())
    rows = []
    for label, tum in series:
        ref, idx, _, _, _, _, aligned = align_to_ground_truth(read_tum(tum), ds, 0.25)
        ref_xyz = ref.positions[idx]
        err = np.linalg.norm(aligned - ref_xyz, axis=1)
        dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref_xyz, axis=0), axis=1))])
        rte = relative_translation_errors(ref_xyz, aligned, RTE_DELTA_M)
        rows.append({'label': label, 'aligned': aligned, 'err': err, 'dist': dist,
                     'ate': float(np.sqrt(np.mean(err ** 2))),
                     'rte': float(np.sqrt(np.mean(rte ** 2))) if rte.size else float('nan'),
                     'matched': int(idx.size)})
    gt = read_tum(ds['ground_truth']).positions
    length = float(np.sum(np.linalg.norm(np.diff(gt, axis=0), axis=1)))
    return ds['sequence'], meta, gt, length, rows


def fmt_m(v):
    return f'{v * 100:.1f} cm' if v < 1 else f'{v:.2f} m'


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True, help='PNG to write')
    parser.add_argument('--title', default='PPBv2 VIO: estimates vs ground truth')
    parser.add_argument('--panel', nargs='+', action='append', required=True,
                        metavar=('DATASET_YAML', 'NAME=TUM'),
                        help='dataset.yaml followed by NAME=estimate.tum entries')
    args = parser.parse_args()

    panels = []
    for spec in args.panel:
        series = [tuple(s.split('=', 1)) for s in spec[1:]]
        if not series or any(len(s) != 2 for s in series):
            parser.error(f'--panel {spec[0]}: give at least one NAME=TUM')
        if len(series) > MAX_SERIES:
            parser.error(f'--panel {spec[0]}: at most {MAX_SERIES} estimates per dataset')
        panels.append(panel_data(spec[0], series))

    n = len(panels)
    fig = plt.figure(figsize=(6.8 * n, 10.2), facecolor=SURFACE)
    grid = fig.add_gridspec(3, n, height_ratios=[3.2, 1.5, 0.55], hspace=0.38, wspace=0.22)
    fig.suptitle(args.title, color=INK, fontsize=13, y=0.985)

    for col, (sequence, meta, gt, length, rows) in enumerate(panels):
        gt_point = ('GPS antenna, position only' if not meta.get('ground_truth_has_orientation')
                    else 'body frame')
        top = fig.add_subplot(grid[0, col])
        style_axes(top)
        top.plot(gt[:, 0], gt[:, 1], color=INK, linewidth=LINE_PT + 0.5, label='ground truth',
                 solid_capstyle='round', zorder=2)
        top.plot(gt[0, 0], gt[0, 1], 'o', markersize=7, color=INK, markeredgecolor=SURFACE,
                 markeredgewidth=1.5, zorder=4)
        top.annotate('start', (gt[0, 0], gt[0, 1]), textcoords='offset points', xytext=(7, -12),
                     color=INK_2, fontsize=8, zorder=5,
                     bbox={'boxstyle': 'round,pad=0.15', 'facecolor': SURFACE, 'edgecolor': 'none'})
        for k, r in enumerate(rows):
            top.plot(r['aligned'][:, 0], r['aligned'][:, 1], color=SERIES[k], linewidth=LINE_PT,
                     label=f"{r['label']}: ATE {fmt_m(r['ate'])}", solid_capstyle='round',
                     solid_joinstyle='round', zorder=3)
        top.set_aspect('equal', adjustable='datalim')
        top.set_xlabel('x [m]')
        top.set_ylabel('y [m]')
        top.set_title(f"{sequence}  ({length:.0f} m; ground truth: {gt_point})",
                      color=INK, fontsize=10, loc='left')
        leg = top.legend(loc='best', fontsize=8, frameon=True, facecolor=SURFACE,
                         edgecolor=GRID, labelcolor=INK_2)
        leg.get_frame().set_linewidth(0.75)

        drift = fig.add_subplot(grid[1, col])
        style_axes(drift)
        for k, r in enumerate(rows):
            drift.plot(r['dist'], r['err'], color=SERIES[k], linewidth=LINE_PT,
                       solid_capstyle='round')
        drift.set_xlabel('distance travelled along ground truth [m]')
        drift.set_ylabel('position error [m]')
        drift.set_ylim(bottom=0)
        drift.set_xlim(left=0)
        drift.set_title('position error after alignment', color=INK_2, fontsize=9, loc='left')

        table_ax = fig.add_subplot(grid[2, col])
        table_ax.axis('off')
        cells = [[r['label'], fmt_m(r['ate']), fmt_m(r['rte']), str(r['matched'])] for r in rows]
        table = table_ax.table(cellText=cells, loc='upper left', cellLoc='left',
                               colLabels=['estimate', 'ATE RMSE', f'RTE per {RTE_DELTA_M:.0f} m',
                                          'matched'],
                               colWidths=[0.42, 0.18, 0.22, 0.18])
        table.auto_set_font_size(False)
        table.set_fontsize(8.5)
        table.scale(1.0, 1.3)
        for (row, _), cell in table.get_celld().items():
            cell.set_edgecolor(GRID)
            cell.set_facecolor(SURFACE)
            cell.get_text().set_color(INK if row else INK_2)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=SURFACE, bbox_inches='tight')
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
