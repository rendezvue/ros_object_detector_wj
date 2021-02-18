# ros_object_detector server & client
object detector by darknet

1. Get source code
<pre><code>catkin_ws/src $ git clone https://github.com/rendezvue/ros_object_detector.git<br>
$ cd ros_object_detector</code></pre>

2. submodule init and update
<pre><code>ros_object_detector$ git submodule update --init --recursive</code></pre>

3. compile darknet 
<pre><code>ros_object_detector$ cd module/darknet/</code></pre>
* Makefile 옵션 : LIBSO=1 로해야 so파일을 생성
* ARCH= 옵션을 시스템에 맞도록 수정 ( rtx3080 의 경우 86 )
<pre><code>ros_object_detector/module/darknet/$ make -j</code></pre>
* 컴파일 완료 후 libdarknet.so 파일이 생성 되었는지 확인

4. catkin_make
<pre><code>catkin_ws $ catkin_make<br>

5. yolo v4 파일 설정 
* /home/fileshare/yolov4_drink_can/can_third 폴더를 src/ros_object_detector 로 복사
* can_third/can_data3/can.data 의 path를 user의 환경에 맞게 수정
* train = /home/hjpark/catkin_ws_yolo/src/ros_object_detector/can_third/can_data3/can_train.txt
* valid = /home/hjpark/catkin_ws_yolo/src/ros_object_detector/can_third/can_data3/can_valid.txt
* names = /home/hjpark/catkin_ws_yolo/src/ros_object_detector/can_third/can_data3/can.names

6. launch 파일 param user 환경에 맞게 수정.
<pre><code>$ catkin_ws/src/ros_object_detector/launch/ros_ensemble_service_server.launch</code></pre> 
* ros_object_detector 폴더 경로
* <arg name="tran_path" default="/home/hjpark/catkin_ws_yolo/src/ros_object_detector" />


<pre><code>$ catkin_ws/src/ros_object_detector/launch/ros_ensemble_service_client.launch</code></pre> 
* ros_object_detector 폴더 경로
* <arg name="tran_path" default="/home/hjpark/catkin_ws_yolo/src/rdv_object_detector" /> 

* 디텍션할 테스트용 이미지 파일
* <arg name="test_image_path_for_object" default="can_third/can_data3/test_images/1.jpg" />

6. run ( 멀티 세션으로 server 와 client 각각 실행 )
<pre><code>$ roslaunch ros_object_detector ros_ensemble_service_server.launch</code></pre>
<pre><code>$ roslaunch ros_object_detector ros_ensemble_service_client.launch</code></pre>


7. 결과
~/.ros/result.png 파일로 이미지 저장 됨.
client 에서는 아래와 같은 결과 메시지 확인 가능.
ros Object Detector client
Service call success
x:2171, y:1812, w:513, h:184, score:0.581027
x:1812, y:1289, w:271, h:232, score:0.780899
x:2083, y:920, w:261, h:126, score:0.835129
x:2248, y:1977, w:513, h:252, score:0.842027
x:1957, y:1851, w:678, h:164, score:0.933522
x:2442, y:775, w:426, h:300, score:0.951298
x:1085, y:1143, w:203, h:242, score:0.976949
x:1793, y:785, w:310, h:756, score:0.980795
x:2045, y:1977, w:591, h:445, score:0.985418
x:1521, y:1851, w:533, h:271, score:0.989173
x:1802, y:2093, w:445, h:358, score:0.992537
x:930, y:1308, w:717, h:746, score:0.994932
x:2035, y:1240, w:368, h:639, score:0.994943
x:1453, y:1502, w:726, h:378, score:0.995706
x:1560, y:1347, w:319, h:203, score:0.996206
x:1046, y:804, w:775, h:378, score:0.996882
x:1211, y:1085, w:746, h:300, score:0.997415
x:1269, y:1269, w:348, h:445, score:0.997481
x:1037, y:1870, w:678, h:600, score:0.997741
x:2520, y:1318, w:484, h:940, score:0.998046
x:2287, y:1221, w:339, h:630, score:0.999150
x:2074, y:949, w:746, h:445, score:0.999784

