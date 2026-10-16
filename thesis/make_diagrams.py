"""Design diagrams for the thesis (system architecture, firmware modes, two-link geometry) -> thesis/figures/*.png"""
import math, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Arc

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figures'); os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({'font.family': 'Arial', 'font.size': 7})
BLUE, ORANGE, AQUA, INK, MUTED = '#2a78d6', '#eb6834', '#1baf7a', '#0b0b0b', '#52514e'
FILLS = {BLUE: '#eef3fb', ORANGE: '#fdf0ea', AQUA: '#e8f7f1', MUTED: '#f4f4f2'}

def box(ax, x, y, w, h, title, lines=(), edge=BLUE):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0,rounding_size=0.12', fc=FILLS[edge], ec=edge, lw=1.1))
    ax.text(x + w / 2, y + h - 0.18, title, ha='center', va='top', fontsize=7.8, fontweight='bold', color=INK)
    for i, s in enumerate(lines):
        ax.text(x + w / 2, y + h - 0.55 - 0.3 * i, s, ha='center', va='top', fontsize=6.6, color=MUTED)

def arrow(ax, p, q, label='', both=False, color=MUTED, at=(0, 0), ha='center'):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle='<|-|>' if both else '-|>', mutation_scale=8, lw=0.9, color=color, shrinkA=0, shrinkB=0))
    if label:
        ax.text((p[0] + q[0]) / 2 + at[0], (p[1] + q[1]) / 2 + at[1], label, ha=ha, va='center', fontsize=6.6, color=INK,
                bbox=dict(fc='white', ec='none', pad=0.6))

# 1. system architecture: signal chain on top, feedback underneath
fig, ax = plt.subplots(figsize=(6.3, 3.7)); ax.set_xlim(0, 14); ax.set_ylim(0, 8.2); ax.axis('off')
box(ax, 0.1, 4.6, 3.7, 3.4, 'Host PC (.NET 10 WPF)', ['Shape generation, IK, checks', 'Time planning -> nodes', 'Upload, workflow, jog', 'Telemetry, experiment records', 'Calibration wizard'])
box(ax, 5.1, 4.6, 3.8, 3.4, 'PSoC 5LP (CY8CKIT-059)', ['Cortex-M3 at 48 MHz', 'Supervision loop (1 ms)', 'Trajectory event-timer ISR', 'SPI for SSI, PWM step pulses', 'HW deadman, pin interrupt', 'Flash trajectory archive'], edge=ORANGE)
box(ax, 10.2, 6.2, 3.7, 1.8, 'DRV8825 drivers x2', ['1/8 microstepping', 'STEP / DIR / EN, nFAULT'])
box(ax, 10.2, 3.6, 3.7, 1.9, 'Steppers + gearboxes', ['J1: NEMA-17, 40:1', 'J2: NEMA-11, 51.06:1'])
box(ax, 10.2, 0.4, 3.7, 2.3, 'Two-link arm', ['L1 = 180 mm, L2 = 176.4 mm', 'Pen at the end of link 2', '(pen lifted by hand)'], edge=MUTED)
box(ax, 5.1, 0.4, 3.8, 2.3, 'AS5045 encoders x2', ['12-bit magnetic encoders', 'on the motor rear shafts', 'one daisy-chained SSI read'], edge=AQUA)
box(ax, 0.1, 0.4, 3.7, 2.3, 'Switches and sensors', ['P0[3] arm1 home switch (NC)', 'P1[0] arm2 home sensor', 'P0[4] global limit (NC)'], edge=MUTED)
arrow(ax, (3.8, 6.3), (5.1, 6.3), 'USB CDC\nor UART', both=True)
arrow(ax, (8.9, 7.1), (10.2, 7.1), 'STEP/DIR')
arrow(ax, (10.2, 6.5), (8.9, 6.5), 'nFAULT')
arrow(ax, (12.05, 6.2), (12.05, 5.5))
arrow(ax, (12.05, 3.6), (12.05, 2.7), 'joint motion', at=(0, 0), ha='center')
arrow(ax, (10.2, 4.1), (8.9, 2.2), 'motor shafts', at=(0.15, 0.1))
arrow(ax, (7.0, 2.7), (7.0, 4.6), 'SSI', at=(0, 0))
arrow(ax, (3.8, 1.6), (5.1, 4.9), 'GPIO / IRQ', at=(-0.1, 0.25))
fig.savefig(os.path.join(OUT, 'fig_system_architecture.png'), dpi=600, bbox_inches='tight'); plt.close(fig)

# 2. firmware modes and what every stop does
fig, ax = plt.subplots(figsize=(6.3, 3.3)); ax.set_xlim(0, 14); ax.set_ylim(0, 7.3); ax.axis('off')
st = {'Disabled': (0.2, 2.9), 'Enabled, idle': (3.6, 2.9), 'Jog': (7.6, 5.2), 'Homing': (7.6, 2.9), 'PTP move': (7.6, 0.6), 'Trajectory': (11.2, 0.6)}
for k, (x, y) in st.items():
    box(ax, x, y, 2.6, 1.2, k, edge=ORANGE if k == 'Disabled' else BLUE)
arrow(ax, (2.8, 3.5), (3.6, 3.5), 'enable', at=(0, -0.3))
arrow(ax, (6.2, 3.9), (7.6, 5.5), 'jog keys', at=(-0.45, 0.15))
arrow(ax, (6.2, 3.5), (7.6, 3.5), 'R', at=(0, 0.3))
arrow(ax, (6.2, 3.1), (7.6, 1.5), 'PTP frame', at=(-0.5, -0.2))
arrow(ax, (10.2, 1.2), (11.2, 1.2), 'RUN', at=(0, 0.3))
ax.text(10.5, 3.5, 'HOME OK sets the\nreference (epoch + 1)', ha='left', va='center', fontsize=6.6, color=MUTED)
ax.add_patch(FancyBboxPatch((0.2, 4.95), 6.6, 2.15, boxstyle='round,pad=0,rounding_size=0.12', fc='#fff7f4', ec=ORANGE, lw=0.9, ls='--'))
ax.text(3.5, 6.95, 'Any stop -> Disabled, reference void', ha='center', va='top', fontsize=7.8, fontweight='bold', color=INK)
for i, s in enumerate(['operator stop (0x79, Space/Esc): drivers off', 'no heartbeat for 250 ms: drivers held',
                       'P0[4] limit, encoder fault, following error: held', 'driver nFAULT: drivers off, latched']):
    ax.text(0.45, 6.5 - 0.36 * i, '- ' + s, ha='left', va='top', fontsize=6.6, color=MUTED)
fig.savefig(os.path.join(OUT, 'fig_firmware_modes.png'), dpi=600, bbox_inches='tight'); plt.close(fig)

# 3. two-link geometry and angle convention
plt.rcParams.update({'font.size': 8})
L1, L2, q1, q2 = 180, 176.38, math.radians(-25), math.radians(40)
e = (L1 * math.cos(q1), L1 * math.sin(q1)); t = (e[0] + L2 * math.cos(q2), e[1] + L2 * math.sin(q2))
fig, ax = plt.subplots(figsize=(5.0, 2.6)); ax.set_aspect('equal'); ax.axis('off')
ax.plot([0, 380], [0, 0], color=MUTED, lw=0.7, ls='--'); ax.text(384, 0, '+X', va='center', fontsize=8, color=MUTED)
ax.plot([e[0], e[0] + 90 * math.cos(q1)], [e[1], e[1] + 90 * math.sin(q1)], color=BLUE, lw=0.8, ls=':')   # link-1 extension
ax.plot([e[0], e[0] + 120], [e[1], e[1]], color=MUTED, lw=0.6, ls=':')
ax.plot([0, e[0]], [0, e[1]], color=BLUE, lw=4, solid_capstyle='round'); ax.plot([e[0], t[0]], [e[1], t[1]], color=ORANGE, lw=4, solid_capstyle='round')
ax.plot(0, 0, 'o', ms=8, mfc='white', mec=INK, mew=1.3); ax.plot(*e, 'o', ms=7, mfc='white', mec=INK, mew=1.3); ax.plot(*t, 'o', ms=5, color=INK)
ax.add_patch(Arc((0, 0), 130, 130, theta1=math.degrees(q1), theta2=0, color=BLUE, lw=1)); ax.text(70, -18, '$q_1$', fontsize=9, color=BLUE)
ax.add_patch(Arc(e, 150, 150, theta1=0, theta2=math.degrees(q2), color=ORANGE, lw=1)); ax.text(e[0] + 78, e[1] + 22, '$q_2$', fontsize=9, color=ORANGE)
ax.add_patch(Arc(e, 80, 80, theta1=math.degrees(q1), theta2=math.degrees(q2), color=INK, lw=0.8)); ax.text(e[0] + 46, e[1] - 2, r'$\theta_e$', fontsize=9, color=INK)
ax.text(e[0] / 2 - 12, e[1] / 2 - 26, '$L_1$ = 180 mm', fontsize=8, color=BLUE, ha='center')
ax.text((e[0] + t[0]) / 2 - 30, (e[1] + t[1]) / 2 + 38, '$L_2$ = 176.4 mm', fontsize=8, color=ORANGE, ha='right')
ax.text(t[0] + 8, t[1] + 2, 'pen (x, y)', fontsize=8, color=INK)
ax.text(0, -16, 'joint 1', fontsize=7.5, color=MUTED, ha='center', va='top'); ax.text(e[0] - 12, e[1] - 12, 'joint 2 (elbow)', fontsize=7.5, color=MUTED, ha='right', va='top')
ax.text(-25, 92, r'$\theta_e = q_2 - q_1$  (elbow angle, turned by motor 2)', fontsize=8, color=INK, ha='left')
ax.set_xlim(-30, 410); ax.set_ylim(-115, 105)
fig.savefig(os.path.join(OUT, 'fig_two_link.png'), dpi=600, bbox_inches='tight'); plt.close(fig)
print('diagrams written')
