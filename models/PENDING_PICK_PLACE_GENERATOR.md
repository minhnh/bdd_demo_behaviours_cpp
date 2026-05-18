# Pending Pick-Place Generator Work

These are the remaining items needed so generated pick-place code does not need manual header edits.

1. Make pose-axis error grouping fully generic.
   - Current generator fixes pose coordinate views by grouping multi-axis pose subobject equality constraints into one `KDL::diff(parent_pose, target_pose)` and projecting the requested components.
   - The same semantic grouping should be generalized for other vector-valued superobjects where applicable: velocity twists, acceleration twists, and wrenches.

2. Avoid emitting unused component locals.
   - Generated grouped pose-diff blocks currently declare all six local component values even when a group only consumes position or only orientation.
   - This compiles, but creates warnings in motions such as place/place-above/retreat. Emit only the components referenced by the group.


4. Remove manual FSM coupling to generated header internals.
   - `pick_place_sim.cpp` must not depend on generated state-only fields like `target_alpha` or `final_target`.
   - Keep the FSM tied to public generated functions and shared error signals only.


6. Revisit transition conditions.
   - The FSM should transition on generated semantic errors plus velocity settling, not on elapsed time alone.
   - Keep this in the harness or generated monitor/event layer, not by editing generated motion headers.
