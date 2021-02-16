# rdv_object_detector
object detector by darknet

1. Get source code
<pre><code>$ git clone https://github.com/rendezvue/rdv_object_detector.git<br>
$ cd rdv_object_detector</code></pre>

2. submodule init and update
<pre><code>rdv_object_detector$ git submodule update --init --recursive</code></pre>

3. compile darknet 
<pre><code>rdv_object_detector$ cd module/darknet/</code></pre>
* Makefile 옵션 : LIBSO=1 로해야 so파일을 생성
* ARCH= 옵션을 시스템에 맞도록 수정
<pre><code>rdv_object_detector/module/darknet/$ make -j</code></pre>
* 컴파일 완료 후 libdarknet.so 파일이 생성 되었는지 확인
<pre><code>rdv_object_detector/module/darknet/$ cd ../../</code></pre>

4. cmake
<pre><code>rdv_object_detector$ mkdir build<br>
rdv_object_detector$ cd build<br>
rdv_object_detector/build$ cmake ..</code></pre>

5. compile
<pre><code>rdv_object_detector/build$ make</code></pre>

4. run
<pre><code>rdv_rs_bag_to_video/build$ ./rdv_object_detector test_input_image_path cfg_path weight_path data_path</code></pre>
* test_input_image_path : 테스트 입력 이미지 path
* cfg_path : yolo 학습 cfg 파일 path
* weight_path : yolo 학습 weight 파일 path
* data_path : yolo 학습 data 파일 path

5. 결과
result.png파일로 저장 됨.
