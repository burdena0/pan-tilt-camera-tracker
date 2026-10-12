"""Generate benchmark metrics, plots and environment metadata."""
from pathlib import Path
import importlib.metadata as metadata
import json
import platform
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parents[1];reports=root/'reports'
report=json.loads((reports/'vision_simulation.json').read_text())
rows={r['case']:r for r in report['results']}
traces=json.loads((reports/'vision_traces.json').read_text())
baseline,nominal=rows['fixed_camera'],rows['nominal']
reduction=100*(1-nominal['mean_center_error_px']/baseline['mean_center_error_px'])
checks={
    'nominal_mean_error_under_5px':nominal['mean_center_error_px']<5,
    'nominal_p95_error_under_10px':nominal['p95_center_error_px']<10,
    'pan_moves_over_20deg':np.ptp(nominal['pan_range_deg'])>20,
    'tilt_moves_over_5deg':np.ptp(nominal['tilt_range_deg'])>5,
    'bird_only_has_no_camera_lock':rows['negative']['observed_locked_frames']==0,
    'watchdog_latches_by_4_25sec':rows['capture_stall']['first_disable_seconds']<=4.25,
    'corrupt_packet_rejected':rows['corrupt_packet']['protocol_rejections']>0,
    'stale_frame_rejected':rows['stale_frame']['protocol_rejections']>0,
    'all_disabled_commands_zero':all(r['nonzero_commands_when_disabled']==0 for r in rows.values()),
    'night_light_and_short_shutter_improve_centering':rows['night_spotlight_short_shutter']['mean_center_error_px']<rows['night_long_shutter']['mean_center_error_px'],
    'night_tracking_mean_under_10px':rows['night_spotlight_short_shutter']['mean_center_error_px']<10,
    'night_tracking_observed_at_least_75percent':rows['night_spotlight_short_shutter']['observed_locked_frames']>=.75*rows['night_spotlight_short_shutter']['frames'],
    'night_bird_only_has_no_camera_lock':rows['night_spotlight_short_shutter_negative']['observed_locked_frames']==0,
}
summary={'mean_error_reduction_percent':reduction,'checks':{k:bool(v) for k,v in checks.items()},
         'all_checks_pass':bool(all(checks.values())),'domain':'these gates apply only to the recorded synthetic scenarios'}
(reports/'acceptance.json').write_text(json.dumps(summary,indent=2))
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,layout='constrained')
for case,color in [('fixed_camera','#8b96a1'),('nominal','#168f91')]:
    t=np.array([r['time'] for r in traces[case]])
    error=np.array([r['error_px'] if r['error_px'] is not None else np.nan for r in traces[case]])
    axes[0].plot(t,error,label=case.replace('_',' '),color=color)
for key,label,color in [('pan','pan','#168f91'),('tilt','tilt','#c28737')]:
    axes[1].plot([r['time'] for r in traces['nominal']],[r[key] for r in traces['nominal']],label=label,color=color)
for case,color in [('night_long_shutter','#8b96a1'),('night_spotlight_short_shutter','#168f91')]:
    axes[2].plot([r['time'] for r in traces[case]],[r['error_px'] for r in traces[case]],label=case.replace('_',' '),color=color)
for ax in axes:ax.legend(loc='upper right');ax.grid(alpha=.2)
axes[0].set_ylabel('centering error (px)');axes[1].set_ylabel('joint angle (deg)');axes[2].set_ylabel('night centering error (px)')
axes[2].set_xlabel('simulation time (s)');fig.suptitle('camera-only CAD simulation / synthetic results')
fig.savefig(reports/'tracking_results.png',dpi=160);plt.close(fig)
packages=['numpy','opencv-contrib-python','onnxruntime','scipy','torch','onnx','mujoco','trimesh','pillow','imageio','imageio-ffmpeg','pyserial','pytest','matplotlib']
environment={'python':platform.python_version(),'os':platform.platform(),'cpu':'AMD Ryzen 7 5800X3D 8-Core Processor',
             'inference_threads':4,'versions':{name:metadata.version(name) for name in packages}}
(reports/'environment.json').write_text(json.dumps(environment,indent=2))
(root/'vision/requirements-verified.txt').write_text('\n'.join(f'{name}=={metadata.version(name)}' for name in packages)+'\n')
print(json.dumps(summary,indent=2))
