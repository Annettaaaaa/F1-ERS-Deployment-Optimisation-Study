# Track data

Input format expected by `physics_models/track.py` (to be written):

- Raw: a centerline as `(x, y)` points (or GPS lat/lon), plus left/right
  track width at each point. Drop raw sources here in `raw/` — e.g. a
  CSV exported from a track-map digitizer, or one of the open track
  files from TUMFTM's `racetrack-database`
  (https://github.com/TUMFTM/racetrack-database), which are already in
  a compatible `x_m,y_m,w_tr_right_m,w_tr_left_m` format and are a good
  starting point before digitizing a real F1 circuit.
- Processed: the centerline resampled to equal arc-length steps and
  converted to `s` (distance along track) and `kappa(s)` (signed
  curvature), plus the track width bounds at each `s`. This is the
  actual input to the distance-domain OCP — everything downstream only
  ever sees `s -> kappa, w_left, w_right`, never raw x/y.

Keep raw and processed separate so the resampling/curvature step is a
reproducible script, not a manual edit.
