## Maps (Bathymetry Data)

This project uses precomputed **bathymetric (depth) maps** provided as grid data:

- **Lakes:** `Bolmen`, `Ringsjön`, `Vombsjön`
- **Sea:** `Öresund`, `Skagerrak`

Each map contains:
- a 2D depth field `Depth(x, y)` (typically in meters; depth is negative in the map convention)
- coordinate axes/scales (`xscale`, `yscale`) for converting grid indices to map coordinates
- a marked **home location** (shown as a small blue bar/jetty), used as the navigation target

Depth measurements `Z_t` are generated/compared by looking up the map value `h(X_t, Y_t)` at the (unknown) boat position.
