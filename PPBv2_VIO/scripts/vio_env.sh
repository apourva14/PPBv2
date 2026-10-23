# Source (do not execute) to set up a shell for one estimator:
#     source /ppbv2_vio/scripts/vio_env.sh mins       # or: openvins
#
# MINS and OpenVINS ship packages with the same names (ov_core, ...), so the two
# workspaces must never be overlaid in one shell.

_vio_estimator="${1:-}"
case "$_vio_estimator" in
  mins)     _vio_ws=/opt/vio/mins_ws/install ;;
  openvins) _vio_ws=/opt/vio/openvins_ws/install ;;
  *)
    echo "usage: source vio_env.sh <mins|openvins>" >&2
    return 2 2>/dev/null || exit 2 ;;
esac

if [ -n "${PPB_VIO_ESTIMATOR:-}" ] && [ "$PPB_VIO_ESTIMATOR" != "$_vio_estimator" ]; then
  echo "this shell is already set up for $PPB_VIO_ESTIMATOR; open a new shell for $_vio_estimator" >&2
  return 1 2>/dev/null || exit 1
fi

# setup.bash scripts reference unset variables; do not run them under `set -u`.
_vio_had_u=0; case $- in *u*) _vio_had_u=1; set +u ;; esac
source /opt/ros/humble/setup.bash
source "$_vio_ws/setup.bash"
if [ -f /ppb_ws/install/setup.bash ]; then
  source /ppb_ws/install/setup.bash
else
  echo "warning: ppb_vio is not built yet; run /ppbv2_vio/scripts/build_ppb_vio.sh" >&2
fi
[ "$_vio_had_u" = 1 ] && set -u

export PPB_VIO_ESTIMATOR="$_vio_estimator"
unset _vio_estimator _vio_ws _vio_had_u
