# todo

  1. Revert the implicit codegen special case:
      - remove support_lift from SnapshotCapture
      - remove <if(snap.support_lift)> + kSupportLift<endif> from emit-snapshot
      - remove kSupportLift from generic runtime unless it is explicitly referenced by generated model data
  2. Make the model explicit, for example conceptually:

     support-lift: LinearDistance = 0.06 m,
     support-z: LinearDistance = Snapshot of <shared.world.pose-elbow-base>.position.z + <spec.support-lift>

     If the DSL does not support arithmetic there yet, then the DSL needs an explicit add/offset expression rather than baking this into codegen.

3. Similarly, if the force is intended to be upward-only and capped, that should be explicit too, not hidden inside generic Impedance. Either the model needs a unilateral constraint/controller option or the controller syntax needs saturation limits.

4. Make pose-axis error grouping fully generic.
   - Current generator fixes pose coordinate views by grouping multi-axis pose subobject equality constraints into one `KDL::diff(parent_pose, target_pose)` and projecting the requested components.
   - The same semantic grouping should be generalized for other vector-valued superobjects where applicable: velocity twists, acceleration twists, and wrenches.

5. Avoid emitting unused component locals.
   - Generated grouped pose-diff blocks currently declare all six local component values even when a group only consumes position or only orientation.
   - This compiles, but creates warnings in motions such as place/place-above/retreat. Emit only the components referenced by the group.

