import argparse
parser=argparse.ArgumentParser();parser.add_argument("--full",action="store_true");parser.add_argument("--output",required=True);args=parser.parse_args()
import os,pty,tty,subprocess,time,threading,struct,json,signal,select
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
rclpy.init();node=rclpy.create_node('lucsys_serial_validation');seen={};subs=[]
for topic,cls in [('odom',Odometry),('imu/data_raw',Imu),('PowerVoltage',Float32)]:
 subs.append(node.create_subscription(cls,topic,lambda m,t=topic:seen.__setitem__(t,m),10))
pub=node.create_publisher(Twist,'/cmd_vel',10)
master,slave=pty.openpty();tty.setraw(slave);device=os.ttyname(slave);running=True;received=bytearray()
frame=bytearray([0x7b,0])+bytearray(struct.pack('>hhh',100,0,0))+bytearray(struct.pack('>hhhhhh',0,0,16384,0,0,0))+bytearray(struct.pack('>H',24000));bcc=0
for b in frame:bcc^=b
frame+=bytes([bcc,0x7d]);assert len(frame)==24
log=open(args.output+'.node.log','w');command=['ros2','launch','lucsys_chassis_driver','lucsys_chassis_driver.launch.py',f'usart_port_name:={device}','car_mode:=LSR550A_mec_arm']
if args.full: command += ['start_ekf:=true','start_state_publisher:=true','model_package:=lucsys_lsr550a_mec_description','model_name:=lsr550a_mec']
proc=subprocess.Popen(command,stdout=log,stderr=log)

def io():
 while running:
  os.write(master,frame)
  if select.select([master],[],[],0.01)[0]:received.extend(os.read(master,4096))
  time.sleep(.02)
idle_end=time.time()+3.2
while time.time()<idle_end:
 rclpy.spin_once(node,timeout_sec=.05)
assert not seen, 'Empty reads must not produce feedback'
assert proc.poll() is None, 'Driver exited during silent serial test'
th=threading.Thread(target=io);th.start()
try:
 commands=[(0.1,0.,0.),(-0.2,0.15,-0.3),(0.,-0.1,0.2),(0.,0.,0.)]
 expected=[]
 for x,y,z in commands:
  end=time.time()+3
  while time.time()<end:
   rclpy.spin_once(node,timeout_sec=.05)
   msg=Twist();msg.linear.x=x;msg.linear.y=y;msg.angular.z=z;pub.publish(msg)
  packet=bytearray([0x7b,0,0])+bytearray(struct.pack('>hhh',int(x*1000),int(y*1000),int(z*1000)))
  checksum=0
  for byte in packet:checksum^=byte
  packet+=bytes([checksum,0x7d]);expected.append(packet.hex())
  assert bytes(packet) in received, packet.hex()
 required={'lucsys_robot'}
 if args.full: required|={'robot_state_publisher','joint_state_publisher','ekf_filter_node'}
 assert required <= set(node.get_node_names()), node.get_node_names()
 assert len(seen)==3,list(seen)
 assert abs(seen['odom'].twist.twist.linear.x-.1)<1e-5
 assert abs(seen['PowerVoltage'].data-24)<1e-5
 result=dict(result='PASS',full=args.full,expected_command_frames=expected,feedback_topics=sorted(seen),voltage=seen['PowerVoltage'].data,odom_velocity=seen['odom'].twist.twist.linear.x,silent_peer='PASS',required_nodes=sorted(required))
 with open(args.output,'w') as output:json.dump(result,output,indent=2)
 print(json.dumps(result))

finally:
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=4)
 except:proc.kill();proc.wait()
 running=False;th.join();node.destroy_node();rclpy.shutdown()
