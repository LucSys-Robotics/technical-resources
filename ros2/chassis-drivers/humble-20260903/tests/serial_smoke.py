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
import tempfile
log=tempfile.TemporaryFile(mode='w+');proc=subprocess.Popen(['ros2','run','lucsys_chassis_driver','lucsys_chassis_node','--ros-args','-p',f'usart_port_name:={device}','-p','car_mode:=LSR550A_mec_arm'],stdout=log,stderr=log)
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
th=threading.Thread(target=io);th.start();end=time.time()+12
try:
 while time.time()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  msg=Twist();msg.linear.x=.1;pub.publish(msg)
 assert len(seen)==3,list(seen)
 assert abs(seen['odom'].twist.twist.linear.x-.1)<1e-5
 assert abs(seen['PowerVoltage'].data-24)<1e-5
 frames=[bytes(received[i:i+11]) for i in range(len(received)-10) if received[i]==0x7b and received[i+10]==0x7d]
 valid=[]
 for f in frames:
  c=0
  for b in f[:9]:c^=b
  if c==f[9] and f[3:5]==b'\x00\x64':valid.append(f.hex())
 assert valid,bytes(received).hex()[:100]
 print(json.dumps(dict(result='PASS',environment='ROS 2 Humble Ubuntu 22.04 arm64; pseudo-terminal, no hardware',empty_read_guard='PASS: silent peer produced no feedback',feedback_topics=list(seen),velocity_command_frames=len(valid),example_frame=valid[0],voltage=seen['PowerVoltage'].data,odom_velocity=seen['odom'].twist.twist.linear.x)))
finally:
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=4)
 except:proc.kill();proc.wait()
 running=False;th.join();node.destroy_node();rclpy.shutdown()
