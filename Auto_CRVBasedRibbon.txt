import maya.cmds as cmds
import math

def create_nurbs_controls_for_driver_joints(driver_jnts, base_name="x_spine"):
    """
    Create NURBS controls on top of driver joints.
    Controls drive joint RST via offsetParentMatrix.
    """
    ctrls = []

    for i, jnt in enumerate(driver_jnts):
        if not jnt or not cmds.objExists(jnt):
            continue

        tok = _pad(i)

        ctrl_name = f"{base_name}_{tok}_ctrl"
        ofs_name  = f"{base_name}_{tok}_ctrl_ofs"

        # offset group
        ofs = cmds.group(em=True, n=ofs_name)

        # nurbs control
        ctrl = cmds.circle(
            n=ctrl_name,
            nr=(1, 0, 0),
            r=2,
            ch=False
        )[0]

        cmds.parent(ctrl, ofs)

        # snap ofs to joint
        cmds.delete(cmds.parentConstraint(jnt, ofs))
        cmds.delete(cmds.scaleConstraint(jnt, ofs))

        # zero ctrl
        cmds.setAttr(ctrl + ".translate", 0, 0, 0)
        cmds.setAttr(ctrl + ".rotate", 0, 0, 0)
        cmds.setAttr(ctrl + ".scale", 1, 1, 1)

        # connect ctrl → joint
        cmds.connectAttr(
            ctrl + ".translate",
            jnt + ".translate",
            f=True
        )
        cmds.connectAttr(
            ctrl + ".rotate",
            jnt + ".rotate",
            f=True
        )


        ctrls.append(ctrl)

    return ctrls


def create_squash_stretch_nodes(sampled_jnts, driver_jnts):
    """
    Create distance-based squash & stretch nodes for the ribbon.

    original_chain: the root joint chain selected by user
    sampled_jnts: list of sampled joints along the curve

    Behavior:
        - Compute distance between first and last original joints
        - Create normalization (Division), power, inversion floatMaths
        - First floatMath output scales Y of all sampled joints
        - Last floatMath output scales X/Z of all sampled joints via plusMinusAverage
    """
    if len(sampled_jnts) < 2:
        raise RuntimeError("sampled_jnts chain must have at least 2 joints.")

    root_jnt = driver_jnts[0]
    end_jnt = driver_jnts[-1]

    # Create squash/stretch nodes
    div_node, inv_div_node = create_distance_normalization_chain(root_jnt, end_jnt)

    for jnt in driver_jnts:
        # ---------------------
        # Y-axis scaling
        # ---------------------
        pma = cmds.createNode("plusMinusAverage", n=jnt + "_scl_pma")
        cmds.setAttr(pma + ".operation", 1)  # Add
        # connect normalized distance to Y
        cmds.connectAttr(div_node + ".outFloat", pma + ".input3D[0]input3Dy", f=True)

        # ---------------------
        # X/Z-axis scaling
        # ---------------------
        # connect inverted power to X/Z
        cmds.connectAttr(inv_div_node + ".outFloat", pma + ".input3D[0].input3Dx", f=True)
        cmds.connectAttr(inv_div_node + ".outFloat", pma + ".input3D[0].input3Dz", f=True)
        
        # connect output to joint scaleY
        cmds.connectAttr(pma + ".output3D", jnt + ".scale", f=True)
        

    return div_node, inv_div_node


def create_distance_normalization_chain(jnt1, jnt2):
    """
    Create distance between two joints (using worldMatrix[0]),
    normalize via Division, raise to power, then divide 1 by the power output.

    Returns:
        div_node: The first floatMath node (normalization)
        final_div_node: The last floatMath node (1 / power)
    """
    base_name = jnt1.replace("_jnt", "")
    
    # Base locator at first joint
    base_loc = cmds.spaceLocator(n=f"{base_name}Base_str_loc")[0]
    cmds.delete(cmds.parentConstraint(jnt1, base_loc))  # snap to first joint
    cmds.setAttr(base_loc + ".translate", *cmds.xform(jnt1, q=True, ws=True, t=True))  # ensure position
    cmds.setAttr(base_loc + ".rotate", 0, 0, 0)
    
    # Tip locator at last joint
    tip_loc = cmds.spaceLocator(n=f"{base_name}Tip_str_loc")[0]
    cmds.delete(cmds.parentConstraint(jnt2, tip_loc))  # snap to last joint
    cmds.setAttr(tip_loc + ".translate", *cmds.xform(jnt2, q=True, ws=True, t=True))
    cmds.setAttr(tip_loc + ".rotate", 0, 0, 0)
    
    

    # Create distance node
    dist_node = cmds.createNode("distanceBetween", n=base_name + "_dist")
    cmds.connectAttr(base_loc + ".worldMatrix[0]", dist_node + ".inMatrix1", f=True)
    cmds.connectAttr(tip_loc + ".worldMatrix[0]", dist_node + ".inMatrix2", f=True)

    # Division normalization
    div_node = cmds.createNode("floatMath", n=base_name + "_nrmlzDiv")
    cmds.setAttr(div_node + ".operation", 3)  # Division
    cmds.connectAttr(dist_node + ".distance", div_node + ".floatA", f=True)
    dist_val = cmds.getAttr(dist_node + ".distance")
    cmds.setAttr(div_node + ".floatB", dist_val)

    # Power node
    pw_node = cmds.createNode("floatMath", n=base_name + "_pw")
    cmds.setAttr(pw_node + ".operation", 6)  # Power
    cmds.connectAttr(div_node + ".outFloat", pw_node + ".floatA", f=True)
    cmds.setAttr(pw_node + ".floatB", 0.5)

    # Final division: 1 / power output
    final_div_node = cmds.createNode("floatMath", n=base_name + "_invPw")
    cmds.setAttr(final_div_node + ".operation", 3)  # Division
    cmds.setAttr(final_div_node + ".floatA", 1)
    cmds.connectAttr(pw_node + ".outFloat", final_div_node + ".floatB", f=True)

    return div_node, final_div_node

def smooth_skin_curve_to_joints(joint_list,curve_shape):
    """
    Skin a curve to an ordered list of joints with smooth, non-linear interpolation:
    - First selected = curve
    - Remaining selections = joints in order
    - First CV = 100% first joint
    - Last CV = 100% last joint
    """
    curve_name = _strip_curve_suffix(curve_shape)
    sc = cmds.skinCluster(joint_list, curve_shape, tsb=True, n=f"{curve_name}_skinCluster")[0]

    spans = cmds.getAttr(curve_shape + ".spans")
    degree = cmds.getAttr(curve_shape + ".degree")
    cv_count = spans + degree
    num_joints = len(joint_list)

    for cv_idx in range(cv_count):
        # first CV = 0, last CV = 1
        t = float(cv_idx) / float(cv_count - 1)
        weights = []

        for j_idx in range(num_joints):
            # joint position normalized along 0..1
            joint_pos = float(j_idx) / float(num_joints - 1)
            distance = abs(t - joint_pos)

            # Cosine falloff, sharp at tips, smooth in between
            w = 0.0
            if cv_idx == 0 and j_idx == 0:
                w = 1.0
            elif cv_idx == cv_count - 1 and j_idx == num_joints - 1:
                w = 1.0
            else:
                # Smooth interpolation using whatever you want, just choose
                falloff = max(0.0, 1.0 - distance * (num_joints - 1))
                #w = 0.5 * (math.cos((1 - falloff) * math.pi) + 1)  # cosine easing
                w = 2*falloff**2 if falloff < 0.5 else -1 + (4 - 2*falloff)*falloff #quadratic ease-in/out
                #w = 4*falloff**3 if falloff < 0.5 else (falloff-1)*(2*falloff-2)**2 + 1 #cubit ease-in/out
                #w = falloff*falloff*(3 - 2*falloff)#Hermite interpolation
                #w = falloff**3*(falloff*(6*falloff - 15) + 10) #smootherstep - more gentle
                #w = math.sin(falloff * math.pi / 2) #spherical smoothing
                #w = falloff**3 #exponential
                #w = 1 - (1-falloff)**3 #exponential ease-out
                #w = 1 / (1 + math.exp(-12*(falloff-0.5))) #logistic curve - Sigmoid
            weights.append(w)

        # normalize weights for inner CVs
        total = sum(weights)
        if total > 0:
            weights = [w / total for w in weights]

        for j_idx, jnt in enumerate(joint_list):
            cmds.skinPercent(sc, f"{curve_shape}.cv[{cv_idx}]", transformValue=[(jnt, weights[j_idx])])

    return sc

def duplicate_chain_for_sampling(root_joint, distance=20):
    """
    Duplicate a joint chain and offset it in Z to use as the ordered ribbon joints.
    Returns the list of duplicated joints in hierarchy order.
    The duplicated joints remain unparented in world space.
    """
    # Duplicate the root joint with hierarchy
    dup_root = cmds.duplicate(root_joint, rc=True, n=root_joint + "_dup")[0]
    
    # Unparent duplicated root to world
    cmds.parent(dup_root, w=True)
    
    # Move the duplicated root in Z (or X/Y as needed)
    cmds.move(0, 0, -distance, dup_root, r=True, ws=True)
    
    # Get all joints in the duplicated chain in hierarchy order
    dup_chain = [dup_root]
    current = dup_root
    while True:
        kids = cmds.listRelatives(current, c=True, type="joint") or []
        if not kids:
            break
        current = kids[0]
        dup_chain.append(current)
        # Unparent each joint to world to avoid hierarchy parenting
        cmds.parent(current, w=True)
    
    return dup_chain


def create_ribbon_curves_from_joints(joint_list, base_name="x_spine", z_offset=0.05):
    """
    Create two curves from the joint list:
    1. base curve: "<base_name>_dfrm_crv" with shape "<base_name>_dfrm_crvShape"
    2. duplicate: "<base_name>_upDfrm_crv" with shape "<base_name>_upDfrm_crvShape"
       slightly offset in Z
    """
    if not joint_list:
        raise RuntimeError("No joints provided.")

    # Get joint positions
    positions = [cmds.xform(j, q=True, ws=True, t=True) for j in joint_list]

    # Create base curve
    dfrm_curve = cmds.curve(p=positions, degree=1, n=f"{base_name}_dfrm_crv")
    shapes = cmds.listRelatives(dfrm_curve, s=True, f=True)
    if shapes:
        cmds.rename(shapes[0], f"{base_name}_dfrm_crvShape")

    # Duplicate curve
    up_curve = cmds.duplicate(dfrm_curve, n=f"{base_name}_upDfrm_crv")[0]
    up_shapes = cmds.listRelatives(up_curve, s=True, f=True)
    if up_shapes:
        cmds.rename(up_shapes[0], f"{base_name}_upDfrm_crvShape")

    # Offset duplicate in Z slightly
    cmds.move(0, 0, z_offset, up_curve, r=True, ws=True)

    return dfrm_curve, up_curve

def _shape_from_sel(obj):
    shapes = cmds.listRelatives(obj, s=True, ni=True) or []
    return shapes[0] if shapes else obj

def _strip_curve_suffix(name):
    for suf in ("_crvShape", "Shape", "shape", "_crv"):
        if name.endswith(suf):
            name = name[:-len(suf)]
    return name.rstrip("_")

def _pad(i, w=2):
    return str(i).zfill(w)

def _extract_joint_chain(root):
    """
    Return ordered list of joints from root downward without unparenting them.
    Parent them to the original root's parent afterwards.
    """
    chain = [root]
    current = root
    while True:
        kids = cmds.listRelatives(current, c=True, type="joint") or []
        if not kids:
            break
        current = kids[0]
        chain.append(current)

    # parent all joints back to the root's parent
    parent_of_root = cmds.listRelatives(root, p=True)
    new_parent = parent_of_root[0] if parent_of_root else None
    for jnt in chain:
        if new_parent:
            cmds.parent(jnt, new_parent)
        else:
            cmds.parent(jnt, w=True)

    return chain


def create_ribbon_for_joints():

    sel = cmds.ls(sl=True) or []
    if len(sel) < 1:
        raise RuntimeError("Select: JointChainRoot.")

    chain_root = sel[0]
    joint_sel = duplicate_chain_for_sampling(sel[0], 25)

    dfrmCRV, upDfrmCRV = create_ribbon_curves_from_joints(joint_sel, base_name="x_spine", z_offset=3)
    
    
    # -------------------------------
    # Extract joint chain
    # -------------------------------
    extracted_chain = _extract_joint_chain(chain_root)
    chain_count = len(extracted_chain)

    if chain_count == 0:
        raise RuntimeError("Could not extract joint chain from: %s" % chain_root)

    # -------------------------------
    # Create start and tip locators for distance measurement Squash&Stretch shieh
    # -------------------------------
    chain_name = _strip_curve_suffix(extracted_chain[0])  # base name from first joint
    
    # -------------------------------
    # Store original Y positions for the pma at the end of the chain
    # -------------------------------
    original_ys = []
    for jnt in extracted_chain:
        y = cmds.getAttr(jnt + ".translateY")
        original_ys.append(y)

    # -------------------------------
    # Curve shapes
    # -------------------------------
    main_shape = _shape_from_sel(dfrmCRV)
    up_shape   = _shape_from_sel(upDfrmCRV)

    main_shape_short = main_shape.split("|")[-1]
    up_shape_short   = up_shape.split("|")[-1]

    main_base = _strip_curve_suffix(main_shape_short)
    up_base   = _strip_curve_suffix(up_shape_short)

    # -------------------------------
    # Curve real param range
    # -------------------------------
    minParam = cmds.getAttr(main_shape + ".minValue")
    maxParam = cmds.getAttr(main_shape + ".maxValue")

    sample_count = len(joint_sel)
    created = []

    # -------------------------------
    # Main loop
    # -------------------------------
    for idx, j in enumerate(joint_sel):

        # compute parameter
        if sample_count == 1:
            param = minParam
        else:
            t = float(idx) / float(sample_count - 1)
            param = minParam + t * (maxParam - minParam)

        tok = _pad(idx)

        dfrm_node = cmds.createNode("pointOnCurveInfo", n=f"{main_base}_{tok}_poncinf")
        up_node   = cmds.createNode("pointOnCurveInfo", n=f"{up_base}_{tok}_poncinf")

        cmds.connectAttr(main_shape + ".worldSpace[0]", dfrm_node + ".inputCurve", f=True)
        cmds.connectAttr(up_shape   + ".worldSpace[0]", up_node   + ".inputCurve", f=True)

        cmds.setAttr(dfrm_node + ".parameter", param)
        cmds.setAttr(up_node   + ".parameter", param)

        # pma
        pma = cmds.createNode("plusMinusAverage", n=f"{main_base}_pma_{tok}")
        cmds.setAttr(pma + ".operation", 2)
        cmds.connectAttr(up_node   + ".positionX", pma + ".input3D[0].input3Dx", f=True)
        cmds.connectAttr(up_node   + ".positionY", pma + ".input3D[0].input3Dy", f=True)
        cmds.connectAttr(up_node   + ".positionZ", pma + ".input3D[0].input3Dz", f=True)

        cmds.connectAttr(dfrm_node + ".positionX", pma + ".input3D[1].input3Dx", f=True)
        cmds.connectAttr(dfrm_node + ".positionY", pma + ".input3D[1].input3Dy", f=True)
        cmds.connectAttr(dfrm_node + ".positionZ", pma + ".input3D[1].input3Dz", f=True)

        # 4by4
        f4 = cmds.createNode("fourByFourMatrix", n=f"{main_base}_4by4_{tok}")

        cmds.connectAttr(pma + ".output3D.output3Dx", f4 + ".in10", f=True)
        cmds.connectAttr(pma + ".output3D.output3Dy", f4 + ".in11", f=True)
        cmds.connectAttr(pma + ".output3D.output3Dz", f4 + ".in12", f=True)

        cmds.connectAttr(dfrm_node + ".normalizedTangentX", f4 + ".in00", f=True)
        cmds.connectAttr(dfrm_node + ".normalizedTangentY", f4 + ".in01", f=True)
        cmds.connectAttr(dfrm_node + ".normalizedTangentZ", f4 + ".in02", f=True)

        cmds.connectAttr(dfrm_node + ".positionX", f4 + ".in30", f=True)
        cmds.connectAttr(dfrm_node + ".positionY", f4 + ".in31", f=True)
        cmds.connectAttr(dfrm_node + ".positionZ", f4 + ".in32", f=True)

        # decompose
        decomp = cmds.createNode("decomposeMatrix", n=f"{main_base}_decompose_{tok}")
        cmds.connectAttr(f4 + ".output", decomp + ".inputMatrix", f=True)

        # rename sampled joint
        try:
            sampled_jnt = cmds.rename(j, f"{main_base}_{tok}_jnt")
        except:
            sampled_jnt = cmds.rename(j, f"{main_base}_{tok}_jnt_#")

        # connect output to that sampled joint
        cmds.connectAttr(decomp + ".outputTranslate", sampled_jnt + ".translate", f=True)
        cmds.connectAttr(decomp + ".outputRotate",    sampled_jnt + ".rotate",    f=True)

        # locator rig
        ofs = cmds.group(em=True, n=f"{main_base}_{tok}_ofs")
        loc = cmds.spaceLocator(n=f"{main_base}_{tok}_loc")[0]
        cmds.parent(loc, ofs)
        
        # snap rotation of the group to the sampled joint on the curve
        cmds.delete(cmds.pointConstraint(sampled_jnt, ofs))
        # snap rotation of the group to the sampled joint on the curve
        cmds.delete(cmds.orientConstraint(sampled_jnt, extracted_chain[idx]))
        
        # leave locator at 0 inside the group
        cmds.setAttr(loc + ".translate", 0, 0, 0)
        cmds.setAttr(loc + ".rotate", 0, 0, 0)
        cmds.setAttr(loc + ".scale", 1, 1, 1)

        # -------------------------------
        # Connect locator → extracted chain joint WITH Y-offset blending
        # -------------------------------
        if idx < len(extracted_chain):
            drive_jnt = extracted_chain[idx]

            # parentConstraint joint → locator
            cmds.parentConstraint(sampled_jnt, loc, mo=False)

            # create plusMinusAverage node to blend locator + original Y
            pma_y = cmds.createNode("plusMinusAverage", n=f"{main_base}_{tok}_pmaY")
            # input3D[0] = locator translate
            cmds.connectAttr(loc + ".translateX", pma_y + ".input3D[0].input3Dx", f=True)
            cmds.connectAttr(loc + ".translateY", pma_y + ".input3D[0].input3Dy", f=True)
            cmds.connectAttr(loc + ".translateZ", pma_y + ".input3D[0].input3Dz", f=True)
            # input3D[1] = original Y
            cmds.setAttr(pma_y + ".input3D[1].input3Dx", 0)
            cmds.setAttr(pma_y + ".input3D[1].input3Dy", original_ys[idx])
            cmds.setAttr(pma_y + ".input3D[1].input3Dz", 0)

            # connect to drive joint
            cmds.connectAttr(pma_y + ".output3D.output3Dx", drive_jnt + ".translateX", f=True)
            cmds.connectAttr(pma_y + ".output3D.output3Dy", drive_jnt + ".translateY", f=True)
            cmds.connectAttr(pma_y + ".output3D.output3Dz", drive_jnt + ".translateZ", f=True)

            # rotation still direct
            cmds.connectAttr(loc + ".rotate", drive_jnt + ".rotate", f=True)

        created.append({
            "sampled_joint": sampled_jnt,
            "drive_joint": extracted_chain[idx] if idx < len(extracted_chain) else None,
            "loc": loc,
            "ofs": ofs
        })

    return created


# Run it
res = create_ribbon_for_joints()

sampled_jnts = [r["sampled_joint"] for r in res]
driver_jnts = [r["drive_joint"] for r in res]
if len(sampled_jnts) < 3:
    raise RuntimeError("Not enough sampled joints to pick first, middle, last.")

# pick first, middle, last
indices = [0, len(sampled_jnts)//2, -1]
skin_jnts = []

for i in indices:
    # duplicate joint
    drv_jnt = cmds.duplicate(
        sampled_jnts[i],
        n=sampled_jnts[i] + "_dfrmCrvSkin"
    )[0]

    cmds.parent(drv_jnt, w=True)  # unparented

    # create offset group
    ofs = cmds.group(em=True, n=drv_jnt + "_ofs")

    # snap ofs to joint
    cmds.delete(cmds.parentConstraint(drv_jnt, ofs))
    cmds.delete(cmds.scaleConstraint(drv_jnt, ofs))

    # parent joint under offset
    #cmds.parent(drv_jnt, ofs)
    
    cmds.connectAttr(drv_jnt + ".translate", ofs + ".translate",f=True)
    cmds.connectAttr(drv_jnt + ".rotate", ofs + ".rotate",f=True)

    # zero joint transforms
    cmds.setAttr(drv_jnt + ".translate", 0, 0, 0)
    cmds.setAttr(drv_jnt + ".rotate", 0, 0, 0)
    cmds.setAttr(drv_jnt + ".scale", 1, 1, 1)

    skin_jnts.append(drv_jnt)


# Skin dfrm curve
smooth_skin_curve_to_joints(skin_jnts, "x_spine_dfrm_crvShape")

# Skin upDfrm curve
smooth_skin_curve_to_joints(skin_jnts, "x_spine_upDfrm_crvShape")


div_node, inv_div_node = create_squash_stretch_nodes(sampled_jnts, driver_jnts)
create_nurbs_controls_for_driver_joints(skin_jnts, base_name="x_spine")
