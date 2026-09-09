# Upstream algorithm snapshot

The headers in `include/dac_sfc/upstream/gcopter` are copied from
`wangyuxuan125/GCOPTER` commit
`ddfe6c710cc80f6686da2906e427acfe52b4a77c`.

They contain the paper-production MINCO/CSGN, Active-Witness corridor and
GCOPTER implementation. `dac_sfc_engine.cpp` isolates the upstream L-BFGS and
root-finder names so this package can coexist with EGO-Planner-v2's optimizer in
one process. See `LICENSE.gcopter` for the upstream MIT license.
