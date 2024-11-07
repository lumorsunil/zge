# zig-game-engine todo

## General

- Rename game engine to something unique

## Physics Engine

### In Progress

- Fixing the R-Tree implementation bugs

### Simulation

- Friction
- Resolving collisions should affect rotational velocity
- Prevent quantum-tunneling for fast moving objects (even reality doesn't fix this!)
    - Maybe this isn't really needed? We could add a maximum velocity so that this is basically not possible because we also can set the simulation time step

### Performance

- Multi-threading
    - Research in how to have a pool of threads ready to run a single update function and then go to sleep until the next update needs to happen
    - Position update can be easily parallellized
        - Dynamic scaling of threads depending on step time measure
            - Number of threads to run = nt
            - Number of vectors in simulation = nv
            - Number of vectors to pass per thread = ceil(nv / nt)
            - We will dynamically increase the number of threads to run based on how much time an update takes
            - Probably need to add a maximum threshold where increasing the number of threads does not improve performance
    - Collision calculations should be possible to parallellize as well, since the calculations do not depend on each other, you could maybe do each level of pages with several threads before descending, if it is even required to improve performance. Maybe the update of the entries can be parallellized? Since that is what is going to probably take up most of the time. We'll take some measurements and see.
