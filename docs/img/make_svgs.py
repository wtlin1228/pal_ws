#!/usr/bin/env python3
"""Generate the tutorial's drawings as static SVG files (python3 docs/img/make_svgs.py)."""
import pathlib

OUT = pathlib.Path(__file__).resolve().parent
INK, QUIET, EDGE, GRID = '#1d2326', '#646d72', '#8b9499', '#c9ced1'
ORANGE, GRAY, RED, GOOD, TINT = '#e76125', '#646d72', '#d21419', '#2e8b57', '#f3f4f5'
FONT = "font-family='Roboto, Helvetica, Arial, sans-serif'"


def svg(w, h, body, label):
    return (f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {w} {h}' width='{w}' height='{h}' role='img' "
            f"aria-label='{label}' {FONT} font-size='13'>\n<defs>"
            f"<marker id='a' viewBox='0 0 10 10' refX='9' refY='5' markerWidth='6' markerHeight='6' orient='auto-start-reverse'><path d='M0 0L10 5L0 10z' fill='{EDGE}'/></marker>"
            f"<marker id='ag' viewBox='0 0 10 10' refX='9' refY='5' markerWidth='6' markerHeight='6' orient='auto-start-reverse'><path d='M0 0L10 5L0 10z' fill='{GRAY}'/></marker>"
            f"</defs>\n<rect width='{w}' height='{h}' fill='white'/>\n{body}\n</svg>\n")


def text(x, y, s, size=13, anchor='start', weight='400', fill=INK):
    return f"<text x='{x}' y='{y}' font-size='{size}' text-anchor='{anchor}' font-weight='{weight}' fill='{fill}'>{s}</text>"


def title(s):
    return text(24, 28, s, 15, weight='600')


def box(cx, cy, w, h, main=False, color=None):
    c = color or ORANGE
    if main:
        return f"<rect x='{cx - w / 2}' y='{cy - h / 2}' width='{w}' height='{h}' rx='8' fill='{c}' fill-opacity='0.12' stroke='{c}' stroke-width='2'/>"
    return f"<rect x='{cx - w / 2}' y='{cy - h / 2}' width='{w}' height='{h}' rx='8' fill='none' stroke='{EDGE}' stroke-width='1.25'/>"


def named(cx, cy, w, h, name, lines, main=False, color=None):
    out = [box(cx, cy, w, h, main, color)]
    if len(lines) == 2:
        out.append(text(cx, cy - 12, name, 13, 'middle', '600'))
        out.append(text(cx, cy + 6, lines[0], 11.5, 'middle', fill=QUIET))
        out.append(text(cx, cy + 22, lines[1], 11.5, 'middle', fill=QUIET))
    else:
        out.append(text(cx, cy - 8, name, 13, 'middle', '600'))
        out.append(text(cx, cy + 12, lines[0], 11.5, 'middle', fill=QUIET))
    return ''.join(out)


def arrow(d, color=EDGE, marker='a'):
    return f"<path d='{d}' fill='none' stroke='{color}' stroke-width='1.25' marker-end='url(#{marker})'/>"


# 1 ---- cell top view ----------------------------------------------------------
rx, ry = 400, 330
b = [title('Cell top view: pallet in front, conveyor to the left, camera above the pallet'),
     f"<circle cx='{rx}' cy='{ry}' r='260' fill='none' stroke='{GRID}' stroke-width='1.25' stroke-dasharray='6 6'/>",
     text(590, 530, 'UR10e reach 1.3 m', 11.5, fill=QUIET),
     f"<rect x='320' y='120' width='160' height='120' rx='4' fill='{ORANGE}' fill-opacity='0.12' stroke='{ORANGE}' stroke-width='2'/>",
     text(496, 150, 'Pallet 0.8 × 0.6 m', 13, weight='600'), text(496, 168, 'center (0, 0.75), top at z = 0.144', 11.5, fill=QUIET),
     text(496, 184, 'frame: pallet_frame', 11.5, fill=QUIET),
     f"<rect x='392' y='96' width='16' height='16' rx='3' fill='{RED}' fill-opacity='0.3' stroke='{RED}' stroke-width='1.25'/>",
     f"<line x1='408' x2='490' y1='104' y2='104' stroke='{GRID}'/>", text(496, 108, '3D camera, z = 2.2 m, looks down', 11.5, fill=QUIET),
     f"<rect x='370' y='300' width='60' height='60' rx='8' fill='none' stroke='{EDGE}' stroke-width='1.25'/>",
     f"<circle cx='{rx}' cy='{ry}' r='14' fill='{EDGE}'/>",
     text(rx, 386, 'UR10e on pedestal', 13, 'middle', '600'), text(rx, 402, 'base_link at (0, 0, 0.6)', 11.5, 'middle', fill=QUIET),
     f"<rect x='170' y='180' width='100' height='300' rx='4' fill='{GRAY}' fill-opacity='0.12' stroke='{GRAY}' stroke-width='2'/>",
     f"<line x1='170' x2='270' y1='330' y2='330' stroke='{GRAY}' stroke-width='1.25' stroke-dasharray='3 3'/>",
     arrow('M220 450V340', GRAY, 'ag'),
     text(160, 334, 'pick point (−0.9, 0)', 11.5, 'end', fill=QUIET),
     text(220, 500, 'Conveyor', 13, 'middle', '600'), text(220, 516, 'x = −0.9, top at z = 0.70', 11.5, 'middle', fill=QUIET),
     text(220, 532, 'boxes flow +y, stop at y = 0', 11.5, 'middle', fill=QUIET),
     arrow('M40 575H100'), arrow('M40 575V515'), text(108, 579, 'x', 11.5, fill=QUIET), text(40, 507, 'y', 11.5, 'middle', fill=QUIET),
     text(40, 596, 'world frame (floor, z up)', 11.5, fill=QUIET)]
(OUT / 'cell-top-view.svg').write_text(svg(760, 616, '\n'.join(b), 'Cell top view'))

# 2 ---- node architecture ------------------------------------------------------
W, H, c1, c2, c3, rA, rB, rC, rD = 200, 72, 128, 380, 632, 128, 232, 336, 440
b = [title('Sensing flows into the executive, commands flow down to MoveIt and ros2_control'),
     f"<rect x='12' y='60' width='232' height='432' rx='8' fill='{TINT}' stroke='{EDGE}'/>", text(28, 82, 'Digital twin (cell_sim)', 13, weight='600'),
     arrow(f'M{c1 + W / 2} {rA}H{c2 - W / 2}'), arrow(f'M{c2} {rA + H / 2}V{rB - H / 2}'), arrow(f'M{c1 + W / 2} {rB}H{c2 - W / 2}'),
     arrow(f'M{c2} {rB + H / 2}V{rC - H / 2}'), arrow(f'M{c2 + W / 2} {rC}H{c3 - W / 2}'), arrow(f'M{c2 - W / 2} {rC}H{c1 + W / 2}'),
     arrow(f'M{c3} {rC + H / 2}V{rD - H / 2}'),
     named(c1, rA, W, H, 'mock_camera', ['world truth → PointCloud2', 'topic /camera/points']),
     named(c1, rB, W, H, 'mock_conveyor', ['moves boxes along +y', '/photo_eye, index_out, feed_in']),
     named(c1, rC, W, H, 'mock_gripper', ['SetSuction srv on/off', 'attach/detach box at TCP']),
     named(c1, rD, W, H, 'world_model', ['truth: boxes, poses, locations', 'markers for RViz'], True),
     named(c2, rA, W, H, 'box_detector', ['height map segmentation', '→ Detection3DArray']),
     named(c2, rB, W, H, 'executive', ['RunJob action, state machine', 'uses cell_planning patterns'], True),
     named(c2, rC, W, H, 'pick_place', ['PickPlace action server', 'approach, grip, lift, place']),
     named(c3, rC, W, H, 'move_group (MoveIt 2)', ['collision-free planning', 'planning scene, IK']),
     named(c3, rD, W, H, 'ros2_control', ['joint_trajectory_controller', 'mock_components joints']),
     text(254, 86, '/camera/points', 11.5, 'middle', fill=QUIET), text(388, 184, '/detections', 11.5, fill=QUIET),
     text(254, 190, '/photo_eye', 11.5, 'middle', fill=QUIET), text(372, 282, 'PickPlace action', 11.5, 'end', fill=QUIET),
     text(506, 294, 'MoveGroup action', 11.5, 'middle', fill=QUIET), text(254, 388, 'SetSuction', 11.5, 'middle', fill=QUIET),
     text(624, 392, 'FollowJointTrajectory', 11.5, 'end', fill=QUIET)]
(OUT / 'architecture.svg').write_text(svg(760, 516, '\n'.join(b), 'Node architecture'))

# 3 ---- ros2_control stack -------------------------------------------------------
cx, W, H, sx, SW, rA, rB, rC, rD, rE = 260, 320, 72, 600, 240, 116, 212, 308, 404, 500
b = [title('A trajectory goes down to the hardware plugin; joint states come back up to TF'),
     arrow(f'M{cx} {rA + H / 2}V{rB - H / 2}'), arrow(f'M{cx} {rB + H / 2}V{rC - H / 2}'), arrow(f'M{cx} {rC + H / 2}V{rD - H / 2}'),
     arrow(f'M{cx} {rD + H / 2}V{rE - H / 2}'), arrow(f'M{cx + W / 2} {rC}H{sx - SW / 2}'),
     named(cx, rA, W, H, 'MoveIt or your code', ['sends a trajectory as a FollowJointTrajectory', 'action goal (joint names, points, times)']),
     named(cx, rB, W, H, 'joint_trajectory_controller', ['interpolates between points every cycle', 'writes a position command per joint']),
     named(cx, rC, W, H, 'controller_manager', ['real-time loop at update_rate (100 Hz here):', 'read → update controllers → write'], True),
     named(cx, rD, W, H, 'hardware interface plugin', ['mock_components/GenericSystem (this tutorial)', 'ur_robot_driver on a real UR']),
     named(cx, rE, W, H, '6 joints declared in the URDF', ['position command interface each', 'position + velocity state interfaces each']),
     named(sx, rC, SW, H, 'joint_state_broadcaster', ['publishes /joint_states', '→ robot_state_publisher → /tf']),
     text(cx + 8, rA + H / 2 + 16, 'FollowJointTrajectory action', 11.5, fill=QUIET), text(cx + 8, rB + H / 2 + 16, 'controller interface', 11.5, fill=QUIET),
     text(cx + 8, rC + H / 2 + 16, 'read() / write()', 11.5, fill=QUIET), text(cx + 8, rD + H / 2 + 16, 'state and command values', 11.5, fill=QUIET),
     text((cx + W / 2 + sx - SW / 2) / 2, rC - H / 2 - 6, 'state interfaces', 11.5, 'middle', fill=QUIET)]
(OUT / 'ros2-control-stack.svg').write_text(svg(760, 560, '\n'.join(b), 'ros2_control stack'))

# 4 ---- detection pipeline -------------------------------------------------------
W, H, c1, c2, c3, rA, rB = 216, 72, 128, 380, 632, 120, 240
b = [title('From point cloud to box poses in six steps; the height map is the pivot'),
     arrow(f'M{c1 + W / 2} {rA}H{c2 - W / 2}'), arrow(f'M{c2 + W / 2} {rA}H{c3 - W / 2}'),
     arrow(f'M{c3} {rA + H / 2}V{(rA + rB) / 2}H{c1}V{rB - H / 2}'), arrow(f'M{c1 + W / 2} {rB}H{c2 - W / 2}'), arrow(f'M{c2 + W / 2} {rB}H{c3 - W / 2}'),
     named(c1, rA, W, H, '1 · Transform', ['cloud → pallet_frame via TF', 'z becomes height above the deck']),
     named(c2, rA, W, H, '2 · Height map', ['1 cm grid over the deck', 'cell value = max z of its points'], True),
     named(c3, rA, W, H, '3 · Top-layer mask', ['all points within 3 cm of the max', 'opened 3 × 3 to cut thin bridges']),
     named(c1, rB, W, H, '4 · Connected components', ['cv::connectedComponents on the mask', 'one blob is one box top']),
     named(c2, rB, W, H, '5 · Rectangle fit', ['cv::minAreaRect per blob', '→ center, size, yaw, fill ratio']),
     named(c3, rB, W, H, '6 · Publish', ['Detection3DArray on /detections', 'plus debug markers for RViz'])]
(OUT / 'detection-pipeline.svg').write_text(svg(760, 300, '\n'.join(b), 'Detection pipeline'))

# 5 ---- executive state machine --------------------------------------------------
W, H, c1, c2, c3, rA, rB, rC, rD = 180, 64, 140, 380, 620, 104, 200, 296, 392
def diamond(cx, cy, label):
    return (f"<polygon points='{cx - 70},{cy} {cx},{cy - 32} {cx + 70},{cy} {cx},{cy + 32}' fill='none' stroke='{EDGE}' stroke-width='1.25'/>"
            + text(cx, cy + 4, label, 13, 'middle'))
b = [title('Depalletizing is a loop: capture, detect, pick, and recover when a stage fails'),
     arrow(f'M{c1 + W / 2} {rA}H{c2 - W / 2}'), arrow(f'M{c2 + W / 2} {rA}H{c3 - 70}'), arrow(f'M{c3} {rA + 32}V{rB - H / 2}'),
     arrow(f'M{c3 + 70} {rA}H720V{rD}H{c3 + W / 2}'), arrow(f'M{c3} {rB + H / 2}V{rC - H / 2}'), arrow(f'M{c3 - W / 2} {rC}H{c2 + 70}'),
     arrow(f'M{c2 - 70} {rC}H{c1 + W / 2}'), arrow(f'M{c2} {rC - 32}V152H160V{rA + H / 2}'), arrow(f'M120 {rC - H / 2}V{rA + H / 2}'),
     arrow(f'M{c1} {rC + H / 2}V{rD - H / 2}'),
     named(c1, rA, W, H, 'CAPTURE', ['trigger camera/capture']), named(c2, rA, W, H, 'DETECT', ['wait one /detections msg']),
     diamond(c3, rA, 'boxes left?'),
     named(c3, rB, W, H, 'SELECT + SCENE', ['highest box, SKU, grasp', 'sync collision objects'], True),
     named(c3, rC, W, H, 'PICK_PLACE', ['send goal, watch stages']), diamond(c2, rC, 'success?'),
     named(c1, rC, W, H, 'RECOVER', ['retreat; retry up to 3']),
     named(c1, rD, W, H, 'FAILED', ['needs a person'], True, RED), named(c3, rD, W, H, 'DONE', ['no boxes detected'], True, GOOD),
     text(c3 + 8, 158, 'yes', 11.5, fill=QUIET), text(705, 96, 'no', 11.5, 'middle', fill=QUIET), text(c3 + 8, 254, 'PickPlace goal', 11.5, fill=QUIET),
     text(490, 286, 'result', 11.5, 'middle', fill=QUIET), text(270, 286, 'no', 11.5, 'middle', fill=QUIET), text(270, 146, 'yes: next box', 11.5, 'middle', fill=QUIET),
     text(128, 204, 'retry', 11.5, fill=QUIET), text(c1 + 8, 350, 'give up', 11.5, fill=QUIET)]
(OUT / 'executive-state-machine.svg').write_text(svg(760, 448, '\n'.join(b), 'Executive state machine'))

# 6 ---- double-pick risk -------------------------------------------------------
L, R, cy = 190, 540, 150
def fbox(cx, cy, w, h):
    return f"<rect x='{cx - w / 2}' y='{cy - h / 2}' width='{w}' height='{h}' rx='3' fill='{ORANGE}' fill-opacity='0.14' stroke='{ORANGE}' stroke-width='1.25'/>"
def pad(cx, cy):
    return f"<rect x='{cx - 50}' y='{cy - 30}' width='100' height='60' rx='2' fill='none' stroke='{EDGE}' stroke-width='2' stroke-dasharray='6 4'/>"
b = [title('A 0.25 m pad on a 0.20 m box overhangs its neighbors: the double-pick risk'),
     fbox(L, cy, 120, 72), pad(L, cy), text(L - 50, cy - 36, 'pad (dashed)', 11.5, fill=QUIET),
     text(L, 206, 'A box 0.30 × 0.18 under a 0.25 × 0.15 pad', 13, 'middle'), text(L, 226, 'risk 0.00: the pad stays on the top face', 11.5, 'middle', fill=QUIET),
     fbox(R - 84, cy, 80, 80), fbox(R, cy, 80, 80), fbox(R + 84, cy, 80, 80),
     f"<rect x='{R - 50}' y='{cy - 30}' width='6' height='60' fill='{RED}' fill-opacity='0.55'/>", f"<rect x='{R + 44}' y='{cy - 30}' width='6' height='60' fill='{RED}' fill-opacity='0.55'/>",
     pad(R, cy), text(R, cy - 48, "red: pad over a neighbor's top", 11.5, 'middle', fill=QUIET),
     text(R, 206, 'three B boxes 0.20 × 0.20 with 1 cm gaps', 13, 'middle'), text(R, 226, 'risk 0.15: 2.5 cm of pad on each neighbor, may lift two boxes', 11.5, 'middle', fill=QUIET)]
(OUT / 'double-pick-risk.svg').write_text(svg(760, 260, '\n'.join(b), 'Double-pick risk'))

# 7 ---- layers A and B ---------------------------------------------------------
s, L, R, cy = 300, 160, 520, 166
def pal(cx):
    return f"<rect x='{cx - 120}' y='{cy - 90}' width='240' height='180' fill='none' stroke='{EDGE}' stroke-width='1.5'/>"
def lbox(cx, x, y, rot):
    w, h = (54, 90) if rot else (90, 54)
    return f"<rect x='{cx + s * x - w / 2}' y='{cy - s * y - h / 2}' width='{w}' height='{h}' rx='3' fill='{ORANGE}' fill-opacity='0.14' stroke='{ORANGE}' stroke-width='1.25'/>"
b = [title('Two layer designs hold the same six boxes; alternating them moves every seam'), pal(L), pal(R)]
for i, (x, y) in enumerate([(-0.16, 0.2), (0.16, 0.2), (-0.16, 0.0), (0.16, 0.0), (-0.16, -0.2), (0.16, -0.2)]):
    b.append(lbox(L, x, y, False)); b.append(text(L + s * x, cy - s * y + 4, str(i + 1), 13, 'middle'))
for i, (x, y, rot) in enumerate([(-0.16, 0.16, False), (0.16, 0.16, False), (-0.3, -0.1, True), (-0.1, -0.1, True), (0.1, -0.1, True), (0.3, -0.1, True)]):
    b.append(lbox(R, x, y, rot)); b.append(text(R + s * x, cy - s * y + 4, str(i + 1), 13, 'middle'))
b += [text(L, 278, 'Layer A: 2 × 3, long side along x', 13, 'middle', '600'), text(L, 296, 'numbers = place order; the robot stands below', 11.5, 'middle', fill=QUIET),
      text(R, 278, 'Layer B: 4 across + 2 along', 13, 'middle', '600'), text(R, 296, 'seams at new x and y, so A and B interlock', 11.5, 'middle', fill=QUIET)]
(OUT / 'layers-a-b.svg').write_text(svg(760, 330, '\n'.join(b), 'Layers A and B'))
print('wrote 7 svg files')
