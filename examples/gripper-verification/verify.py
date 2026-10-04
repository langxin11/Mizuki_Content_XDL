"""Verify the real gripper assets without editing or re-exporting them."""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import platform
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def virtual_assets(path):
    root = ET.parse(path).getroot()
    resources = {}
    manifest = []
    meshdir = root.find('compiler').get('meshdir', '')
    for index, mesh in enumerate(root.findall('./asset/mesh')):
        original = mesh.get('file')
        resource = path.parent / meshdir / original
        # Keep implicit mesh names while replacing only the filesystem lookup.
        mesh.set('name', mesh.get('name', Path(original).stem))
        alias = f'mesh_{index:03d}{resource.suffix}'
        mesh.set('file', alias)
        resources[alias] = resource.read_bytes()
        manifest.append({'file': original, 'sha256': digest(resource)})
    root.find('compiler').set('meshdir', '')
    return root, resources, manifest


def load(root, resources):
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'), resources)


def run(path, output):
    try:
        mujoco.MjModel.from_xml_path(str(path.resolve()))
        direct_load = {'success': True}
    except Exception as error:
        direct_load = {'success': False, 'error': str(error)}
    root, resources, meshes = virtual_assets(path)
    original_model = load(root, resources)
    original_data = mujoco.MjData(original_model)
    mujoco.mj_forward(original_model, original_data)
    pairs = [(c.get('site1'), c.get('site2')) for c in root.findall('./equality/connect')]
    assert len(pairs) == 4 and original_model.nu == 1
    actuator = root.find('actuator')[0]
    assert actuator.tag == 'motor' and actuator.get('joint') == 'gripper_drive'
    initial = [float(np.linalg.norm(original_data.site(a).xpos - original_data.site(b).xpos)) for a, b in pairs]
    base = root.find('./worldbody/body[@name="base"]')
    free = base.find('freejoint')
    assert free is not None and free.get('name') == 'base_freejoint'
    base.remove(free)
    model = load(root, resources)
    data = mujoco.MjData(model)
    duration, period, amplitude, kp, kd = 20.0, 10.0, 1.2, 5.0, 0.2
    dt = float(model.opt.timestep)
    qaddr = model.joint('gripper_drive').qposadr[0]
    vaddr = model.joint('gripper_drive').dofadr[0]
    left = model.joint('left_finger_slide').qposadr[0]
    right = model.joint('right_finger_slide').qposadr[0]
    sid_left = model.site('left_taxel_11').id
    sid_right = model.site('right_taxel_11').id
    rows = []
    for _ in range(round(duration / dt)):
        time = float(data.time)
        phase = 2 * np.pi * time / period
        target = amplitude / 2 * (1 - np.cos(phase))
        velocity = amplitude / 2 * 2 * np.pi / period * np.sin(phase)
        data.ctrl[0] = np.clip(kp * (target - data.qpos[qaddr]) + kd * (velocity - data.qvel[vaddr]), *model.actuator_ctrlrange[0])
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all() and np.isfinite(data.qacc).all()
        residuals = [float(np.linalg.norm(data.site(a).xpos - data.site(b).xpos)) for a, b in pairs]
        normal_left = data.site_xmat[sid_left].reshape(3, 3)[:, 2]
        normal_right = data.site_xmat[sid_right].reshape(3, 3)[:, 2]
        parallel = np.degrees(np.arccos(np.clip(abs(np.dot(normal_left, normal_right)), 0, 1)))
        spacing = np.linalg.norm(data.site_xpos[sid_left] - data.site_xpos[sid_right])
        rows.append([float(data.time), target, float(data.qpos[qaddr]), float(data.qpos[left]), float(data.qpos[right]), *residuals, float(parallel), float(spacing), float(data.ctrl[0]), float(data.ncon)])
    samples = np.asarray(rows)
    warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(data.warning) if w.number}
    result = {
        'model': path.name, 'xml_sha256': digest(path), 'mesh_count': len(meshes), 'meshes': meshes, 'direct_file_load': direct_load,
        'original_nq': original_model.nq, 'fixed_base_nq': model.nq, 'actuator_count': model.nu,
        'actuator': dict(actuator.attrib), 'equality_count': model.neq, 'initial_closure_m': initial,
        'runtime_changes': ['Remove base_freejoint in memory to attach base to world; no other MJCF changes.'],
        'duration_s': float(data.time), 'timestep_s': dt, 'solver': mujoco.mjtSolver(model.opt.solver).name,
        'drive_reference': {'range_rad': [0, amplitude], 'period_s': period, 'kp_Nm_per_rad': kp, 'kd_Nm_s_per_rad': kd},
        'maximum_closure_mm': (samples[:, 5:9].max(axis=0) * 1000).tolist(),
        'rms_closure_mm': (np.sqrt(np.mean(samples[:, 5:9] ** 2, axis=0)) * 1000).tolist(),
        'maximum_parallel_angle_deg': float(samples[:, 9].max()),
        'maximum_slider_difference_mm': float(np.abs(samples[:, 3] - samples[:, 4]).max() * 1000),
        'slider_travel_mm': ((samples[:, 3:5].max(axis=0) - samples[:, 3:5].min(axis=0)) * 1000).tolist(),
        'maximum_drive_tracking_error_rad': float(np.abs(samples[:, 1] - samples[:, 2]).max()),
        'maximum_contacts': int(samples[:, 12].max()), 'warnings': warnings,
        'criteria': {'maximum_closure_mm': 0.1, 'maximum_parallel_angle_deg': 0.1, 'minimum_slider_travel_mm': 5.0, 'warnings': 0},
    }
    result['empty_load_test_passed'] = bool(np.max(result['maximum_closure_mm']) < 0.1 and result['maximum_parallel_angle_deg'] < 0.1 and min(result['slider_travel_mm']) > 5 and not warnings)
    np.savetxt(output / (path.stem + '.csv'), samples, delimiter=',', header='time_s,target_rad,drive_rad,left_slide_m,right_slide_m,left_center_m,left_axis_m,right_center_m,right_axis_m,parallel_deg,center_site_distance_m,torque_Nm,contacts', comments='')
    return result, samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--source-commit', required=True)
    args = parser.parse_args()
    assert re.fullmatch('[0-9a-f]{40}', args.source_commit)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {'checked': datetime.now(timezone(timedelta(hours=8))).date().isoformat(), 'python': platform.python_version(), 'system': platform.system(), 'mujoco': mujoco.__version__, 'numpy': np.__version__, 'source_commit': args.source_commit,
              'loader': 'MuJoCo VFS with ASCII aliases for unchanged mesh bytes; direct file loading is checked separately per model.',
              'scope': 'Compile real assets and test fixed-base empty-load motion; no CAD re-export, external-object contact, hardware or tactile calibration validation.', 'models': []}
    plot_samples = None
    for name in ['parallel_gripper.xml', 'parallel_gripper_prepared.xml', 'parallel_gripper_height_sphere_collision.xml']:
        result, samples = run(args.asset_dir / name, args.output_dir)
        report['models'].append(result)
        if name == 'parallel_gripper.xml': plot_samples = samples
    (args.output_dir / 'verification.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = plot_samples[:, 0]
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), layout='constrained')
    axes[0].plot(t, plot_samples[:, 1], label='reference')
    axes[0].plot(t, plot_samples[:, 2], label='drive')
    axes[0].set_ylabel('Drive angle (rad)')
    axes[0].legend()
    axes[1].plot(t, plot_samples[:, 3] * 1000, label='left')
    axes[1].plot(t, plot_samples[:, 4] * 1000, '--', label='right')
    axes[1].set_ylabel('Slide displacement (mm)')
    axes[1].legend()
    for k, label in enumerate(['left center', 'left axis', 'right center', 'right axis']):
        axes[2].plot(t, plot_samples[:, 5 + k] * 1000, label=label)
    axes[2].set_ylabel('Closure residual (mm)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(ncol=2)
    for ax in axes: ax.grid(alpha=0.3)
    fig.suptitle(f'Fixed-base empty-load check / MuJoCo {mujoco.__version__}')
    fig.savefig(args.output_dir / 'empty-load-validation.png', dpi=180)
    plt.close(fig)
    print(json.dumps([{'model': m['model'], 'max_closure_mm': max(m['maximum_closure_mm']), 'travel_mm': m['slider_travel_mm'], 'parallel_deg': m['maximum_parallel_angle_deg'], 'contacts': m['maximum_contacts'], 'passed': m['empty_load_test_passed'], 'warnings': m['warnings']} for m in report['models']], ensure_ascii=False))
    if not all(m['empty_load_test_passed'] for m in report['models']):
        raise SystemExit('At least one model failed the declared empty-load criteria; inspect verification.json.')


if __name__ == '__main__':
    main()
