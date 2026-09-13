import hashlib,json,os,random,sys,time,traceback
from pathlib import Path
import cv2,numpy as np,trimesh,torch
from rm75_app.perception.rrtrack.foundationpose_adapter import FoundationPoseRefiner
from rm75_app.perception.rrtrack.models import FrameObservation
out=Path(__file__).resolve().parent
root=Path('/home/zhangzhao/PycharmProjects/FoundationPose')
data=root/'demo_data/mustard0'
report=dict(domain='offline_recorded_RGBD_model_inference',seed=0,hardware_connected=False,
            checkpoint_admitted=False,capture_time=None,world_extrinsic=None,
            unknown_reason='Exposure provenance and RM75 world extrinsic are not established for this example')
try:
    random.seed(0);np.random.seed(0);torch.manual_seed(0)
    rgb_path=sorted((data/'rgb').glob('*.png'))[0]
    depth_path=data/'depth'/rgb_path.name;mask_path=data/'masks'/rgb_path.name
    mesh_path=data/'mesh/textured_simple.obj'
    files=[rgb_path,depth_path,mask_path,data/'cam_K.txt',mesh_path,
           data/'mesh/textured_simple.obj.mtl',data/'mesh/texture_map.png',
           root/'weights/2024-01-11-20-02-45/model_best.pth',
           root/'weights/2023-10-28-18-33-37/model_best.pth']
    report['input_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    rgb=cv2.cvtColor(cv2.imread(str(rgb_path)),cv2.COLOR_BGR2RGB)
    depth=cv2.imread(str(depth_path),-1).astype(np.float32)/1000.
    mask=cv2.imread(str(mask_path),-1)
    if mask.ndim==3:mask=np.any(mask>0,axis=2)
    else:mask=mask>0
    frame=FrameObservation(rgb,depth,np.loadtxt(data/'cam_K.txt').reshape(3,3),frame_index=0,timestamp_s=None)
    mesh=trimesh.load(mesh_path,force='mesh')
    report.update(frame_sequence=0,frame_file=rgb_path.name,depth_unit='metres_from_original_reader_millimetre_PNG',
                  mesh_extents=mesh.extents.tolist(),object_frame='original_example_mesh',
                  inference_started_at=time.time(),versions={'torch':torch.__version__,'numpy':np.__version__})
    os.chdir(root)
    refiner=FoundationPoseRefiner(mesh,foundationpose_root=root,debug_dir=out/'debug')
    result=refiner.global_register(frame,mask)
    if result is None:raise RuntimeError('No model pose returned')
    report.update(status='MODEL_INFERRED_NOT_SWM_CHECKPOINT_QUALIFIED',
        T_camera_object=np.asarray(result.T_cam_obj).tolist(),quality_score=result.score,source=result.source)
except BaseException as exc:
    report.update(status='FAILED',error=f'{type(exc).__name__}: {exc}')
    (out/'traceback.txt').write_text(traceback.format_exc())
finally:
    report['inference_completed_at']=time.time()
    (out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps(report),flush=True)
sys.exit(1 if report['status']=='FAILED' else 0)
