2D Physics Engine I've been developing using OpenGL. Set up to work on Polygon-Polygon.

TODO (Correctness
- Fix restitution application so bounce is consistent for all collisions, not only cases with significant angular impulse.
(Currently restitution appears to depend on angled/rotational contact response.)

TODO (Performance):
- Add broad-phase partitioning (spatial hash / uniform grid or sweep-and-prune) to reduce O(n^2) pair checks.
- Add shape-specific collision paths (Circle–Circle, Circle–Polygon, Box–Box) instead of treating everything as generic polygons. This avoids expensive SAT for common cases.

https://github.com/user-attachments/assets/12808075-fe2f-4803-b852-b403fb83c422

Demo 



