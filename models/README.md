# Ghost filter models

The default models of the driver's ghost filter (`ghost_filter.*`), installed
to `share/multiscan_driver/models/`. Parameters name them without path or
extension (`model_file: artifact_full`).

| File | Inputs | Trained | Scans / returns / artifacts | Average precision | At threshold 0.3 |
|---|---|---|---|---|---|
| `artifact_full.model.txt` | range and intensity | 2026-11-18 | 419 / 1.75 M / 4,664 | 0.790 | precision 0.69, recall 0.77, 12.4 flagged per scan |
| `artifact_xyz.model.txt` | range only | 2026-11-18 | 711 / 3.12 M / 5,876 | 0.754 | precision 0.67, recall 0.74, 9.1 flagged per scan |

Both are realtime models (they look at the two previous scans only), with 600
trees and early exit checkpoints. The figures are cross-validated against the
labels, from the `.report.json` next to each model. That report also lists
the bags and scans the model was trained on. "Precision" counts only labelled
points, so it is a lower bound: many flagged points that weren't labelled are
ghosts too.

## Where they come from

The models are trained with [ghost-ml](https://github.com/S1ink/ghost-ml),
a separate project (not part of the LANCE workspace). Its `annotator_app`
labels recorded bags and writes `artifact_<variant>.model.txt` next to every
model it trains. The format is described in its `export_model.py`. You only
need ghost-ml to train new models, not to use these.

To replace these models with newly trained ones, publish them from ghost-ml:
use "Publish to driver" in the app, or run
`annotator_app/publish_model.py --driver <this repository>`. Then rebuild
the driver and run its tests (`test_ghost_filter` checks that the defaults
load and fit the sensor). Finally, commit the models here.
