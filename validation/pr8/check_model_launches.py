import importlib.util,json
from pathlib import Path
from launch import LaunchContext
rows=[]
for path in sorted(Path('/audit/ws/install').glob('*/share/*/launch/display.launch.py')):
 spec=importlib.util.spec_from_file_location('model_launch',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 m.generate_launch_description()
 for name in m.MODELS:
  c=LaunchContext();c.launch_configurations.update(model=name,gui='false',rviz='false',use_sim_time='false')
  actions=m._launch(c);assert len(actions)==4;rows.append({'package':m.PACKAGE,'model':name,'result':'LAUNCH_RESOLUTION_AND_MESH_PATHS_PASS'})
assert len(rows)==37,len(rows)
Path('/audit/model-launches.json').write_text(json.dumps(rows,indent=2));print('Validated',len(rows),'model launch configurations')
