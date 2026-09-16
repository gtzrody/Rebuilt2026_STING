# YAMS removal / mechanism redesign

The material-handling mechanisms were redesigned as rollers/conveyors rather than Arms.

## Mechanism classifications
- Shooter: flywheel / velocity-controlled mechanism.
- Hood: true rotational arm mechanism; retains position control and soft limits.
- Intake: roller/open-loop duty-cycle mechanism.
- Indexer: roller/conveyor/open-loop duty-cycle mechanism.
- Hopper: roller/conveyor/open-loop duty-cycle mechanism.

The original YAMS Intake, Indexer, and Hopper each used `Arm`, but the robot API only ever commanded `set(double)` motor output. Their old 3 ft / 1 lb arm simulation values therefore were not representative of the physical mechanism and have been removed.

The replacement classes preserve the existing `set(double)` command API used by RobotContainer and ShootCommand/FeedCommand.

The new roller simulations use a small generic rotational inertia only to provide basic simulated motor motion; it is not intended to be a measured physical model. For high-fidelity simulation, replace the inertia with an experimentally measured roller/conveyor inertia.
