from cv2 import aruco
import cv2
import numpy as np
import time
import rospy
from mavros_msgs.msg import OverrideRCIn
from RealSense import RealSense
import datetime
import os
 
camera=RealSense()
print('Detected start')
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# catkin_ws/src/test/srcに保存するためディレクトリとファイルを指定
output_directory = "/home/ubuntu/catkin_ws/src/test/src/picture"
file_prefix = "captured_image_"  # 画像ファイルの接頭辞

 
rospy.init_node('forward_and_turn')
pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
rate = rospy.Rate(100)  # ループの実行頻度（10 Hz）
 
# サーボチャンネルの設定
servo_channel_forward = 1
servo_channel_reverse = 3
forward_value = 1150  # 前進のサーボ値
reverse_value = 1500  # 中立のサーボ値（停止）
back_value = 1800     # 後進のサーボ値
left_turn_value = 1200     # 左に90度回転するサーボ値
right_turn_value = 1800     # 右に90度回転するサーボ値
 
# マーカーまでの最小距離（30cm）を設定
min_distance = 500  # メートルじゃないよ
 
start_time = time.time()

def capture_image(frame):
    # 現在時刻を取得
    now = datetime.datetime.now()
    timestamp = now.strftime('%Y%m%d_%H%M%S')
    # ファイル名を生成
    filename = f"{file_prefix}{timestamp}.jpg"
    filepath = os.path.join(output_directory, filename)
    # 画像を保存
    cv2.imwrite(filepath, frame)
    print('captured image saved')

# time秒止まる関数
def stop(second):
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    time.sleep(second)
 
# 旋回中に画像を取得して壁があったら旋回を停止する関数
def detection_turn_left(detection_thereshold):
    # 左に90度回転
    start_time = time.time()
    while (1):
        frame,depth,corners,list_ids = get_camera()
        capture_image(frame)
        depth_top_left = depth[:int(frame.shape[0] / 2), :int(frame.shape[1] / 6)]
        avg_depth_top_left = np.mean(depth_top_left)
        if time.time()-start_time > 0.1:
            frame,depth,corners,list_ids = get_camera()
            depth_top_left = depth[:int(frame.shape[0] / 2), :int(frame.shape[1] / 6)]
            avg_depth_top_left = np.mean(depth_top_left)
            if avg_depth_top_left < detection_thereshold:
                    print("左に壁があります")
                    stop(1.0)
                    turn_right(1.0)
                    break
            start_time = time.time()
        # 左に旋回
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = left_turn_value
        pub.publish(msg)
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    time.sleep(2)
    
    
    # while True:
    #     stop(1.0)
    #     frame,depth,corners,list_ids = get_camera()
    #     depth_top_left = depth[:int(frame.shape[0] / 2), :int(frame.shape[1] / 6)]
    #     avg_depth_top_left = np.mean(depth_top_left)
    #     if avg_depth_top_left < detection_thereshold:
    #         stop(1.0)
    #         return 0
        
    #     # 左に旋回
    #     msg = OverrideRCIn()
    #     msg.channels[servo_channel_forward - 1] = reverse_value
    #     msg.channels[servo_channel_reverse - 1] = left_turn_value
    #     pub.publish(msg)
    #     print("えーい")
    #     time.sleep(0.1)
            
            
def detection_turn_right(detection_thereshold):
      # 右に90度回転
    start_time = time.time()
    while (1):
        frame,depth,corners,list_ids = get_camera()
        capture_image(frame)
        depth_top_left = depth[:int(frame.shape[0] / 2), :int(frame.shape[1] / 6)]
        avg_depth_top_left = np.mean(depth_top_left)
        if time.time()-start_time > 0.2:
            frame,depth,corners,list_ids = get_camera()
            capture_image(frame)
            depth_top_left = depth[:int(frame.shape[0] / 2), :int(frame.shape[1] / 6)]
            avg_depth_top_left = np.mean(depth_top_left)
            if avg_depth_top_left < detection_thereshold:
                    print("右に壁があります")
                    stop(1.0)
                    turn_left(1.0)
                    break
            start_time = time.time()
        # 左に旋回
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = right_turn_value
        pub.publish(msg)
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    time.sleep(2)
 
# 壁を判別して真っ直ぐ進む関数
 
def wall_detection(frame, depth):
    # 画面上半分に対して左1/6の深度情報を取得
    depth_top_left = depth[:int(frame.shape[0] / 2), :int(frame.shape[1] / 6)]
    # 画面上半分に対して左1/3の深度情報を取得
    depth_top_left_hard = depth[:int(frame.shape[0] / 2), :int(frame.shape[1] / 3)]
    # 画面上半分に対して右1/6の深度情報を取得
    depth_top_right = depth[:int(frame.shape[0] / 2), int(frame.shape[1] / 6 * 5):]
    # 画面上半分に対して右1/3の深度情報を取得
    depth_top_right_hard = depth[:int(frame.shape[0] / 2), int(frame.shape[1] / 3 * 2):]
    # 画面上半分に対して中央1/3の深度情報を取得
    depth_top_center = depth[:int(frame.shape[0] / 2), int(frame.shape[1] / 3):int(frame.shape[1] / 3 * 2)]
    # 平均を取得
    avg_depth_top_left = np.mean(depth_top_left)
    avg_depth_top_right = np.mean(depth_top_right)
    avg_depth_top_left_hard = np.mean(depth_top_left_hard)
    avg_depth_top_right_hard = np.mean(depth_top_right_hard)
    avg_depth_top_center = np.mean(depth_top_center)

    if avg_depth_top_center < 300:
        print("前方に壁があります.少しバックします．")
        # 少しバックする
        start_time = time.time()
        
        while time.time() - start_time < 0.8:
            msg = OverrideRCIn()
            msg.channels[servo_channel_forward - 1] = back_value
            msg.channels[servo_channel_reverse - 1] = reverse_value
            pub.publish(msg)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = forward_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.5)
 
    # 平均深度が1m未満なら壁があると判断
    # hardじゃない方は少し角度をずらす（閾値：300mm）
    # 左にあった場合
 
    if avg_depth_top_left < 600 and avg_depth_top_left_hard > 1000:
        print("左に壁があります.少し角度をずらします")
        
        # 画像に描画
        cv2.rectangle(frame, (0, 0), (int(frame.shape[1] / 6), int(frame.shape[0] / 2)), (255, 0, 0), 2)
        cv2.putText(frame, "LEFT WALL", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), thickness=2)
        
        capture_image(frame)
        turn_right(0.5)
 
    # 右にあった場合
 
    elif avg_depth_top_right < 600 and avg_depth_top_right_hard > 1000:
        # 画像に描画
        cv2.rectangle(frame, (int(frame.shape[1] / 6 * 5), 0), (frame.shape[1], int(frame.shape[0] / 2)), (255, 0, 0), 2)
        cv2.putText(frame, "RIGHT WALL", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), thickness=2)
        print("右に壁があります.少し角度をずらします")
        capture_image(frame)
        
        turn_left(0.5)

    # hardな方は大きく角度をずらす（閾値：1m）
    # まず右にあった場合
 
    elif avg_depth_top_left_hard < 400:
        print("左に壁があります.大きく角度をずらします")
        # 画像に描画
        cv2.rectangle(frame, (0, 0), (int(frame.shape[1] / 3), int(frame.shape[0] / 2)), (0, 0, 255), 2)
        cv2.putText(frame, "LEFT WALL", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), thickness=2)
 
        capture_image(frame)
        turn_right(0.8)
 
    # 左にあった場合
 
    elif avg_depth_top_right_hard < 400:
        print("右に壁があります.大きく角度をずらします")
        # 画像に描画
        cv2.rectangle(frame, (int(frame.shape[1] / 3 * 2), 0), (frame.shape[1], int(frame.shape[0] / 2)), (0, 0, 255), 2)
        cv2.putText(frame, "RIGHT WALL", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), thickness=2)
        capture_image(frame)
        turn_left(0.8)
        
    else:
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = forward_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.5)
 
    capture_image(frame)
 
# 時間で旋回する関数
 
def turn_left(turn_time):
    print("左に少し旋回")
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    time.sleep(1)

    # 右にちょっと回転
    start_time = time.time()
    while time.time() - start_time < turn_time:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = left_turn_value
        pub.publish(msg)
        time.sleep(0.1)
 
    start_time = time.time()
    while time.time() - start_time < 0.3:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.1)
    time.sleep(1)
 
def turn_right(turn_time):
    print("右に少し旋回")
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    time.sleep(1)
    
    # 右にちょっと回転
    start_time = time.time()
    while time.time() - start_time < turn_time:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = right_turn_value
        pub.publish(msg)
        time.sleep(0.1)
    start_time = time.time()
    
    while time.time() - start_time < 0.3:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.1)
    time.sleep(1)
 
# 時間で直進
def forward(turn_time):
    print("少し旋回",time)
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    time.sleep(1)
 
    # 右にちょっと回転
    start_time = time.time()
    while time.time() - start_time < turn_time:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = forward_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.1)
    start_time = time.time()
 
    while time.time() - start_time < 0.3:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.1)
    time.sleep(1)
   
def back(turn_time):
    print("少しバック",time)
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    time.sleep(1)
    
    # 右にちょっと回転
    start_time = time.time()
    while time.time() - start_time < turn_time:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = back_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.1)
    start_time = time.time()
 
    while time.time() - start_time < 0.3:
        print(time.time() - start_time)
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = reverse_value
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        time.sleep(0.1)
    time.sleep(1)
 
def find_point_P(A, B, d):
    M = np.add(A, B) / 2
    AB = np.subtract(B, A)
    orthogonal_vector = np.array([-AB[1], AB[0], 0])
    
    if np.linalg.norm(orthogonal_vector) == 0:
        orthogonal_vector = np.array([1, 0, 0])
    orthogonal_vector = (orthogonal_vector / np.linalg.norm(orthogonal_vector))*d
 
    P_up = np.add(M, orthogonal_vector)
    P_down = np.subtract(M, orthogonal_vector)
    
    return tuple(M), tuple(P_up), tuple(P_down)
 
 
 
def station_detection(frame, depth, corners, list_ids):
 
    while True:
        frame, depth = camera.get_frame()
        corners, ids, rejectedImgPoints = detector.detectMarkers(frame)
        list_ids = np.ravel(ids)

        if len(list_ids) != 2:
            
            return 1
 
        print(">>>", list_ids)
 
        # ArUcoマーカが検出されたら赤い丸を描画する
 
        if ids is not None:
            z0 = 0
            z1 = 0
 
            for i in [3,4]:
                marker_index = np.where(list_ids == i)[0][0]  # マーカー1のインデックスを取得
                marker_corners = corners[marker_index][0]
 
                # マーカーの中心座標を計算
                marker_center = np.mean(marker_corners, axis=0)
                marker_center_x = np.mean(marker_corners[:, 0])
                marker_center_y = np.mean(marker_corners[:, 1])
 
                # マーカーまでの距離を計算
                marker_depth = depth[int(marker_center[1]), int(marker_center[0])]
                # マーカーまでの距離をArUcoマーカの面全体の平均深度で取得
                # marker_depth = np.mean(depth[int(marker_corners[0][1]):int(marker_corners[2][1]), int(marker_corners[0][0]):int(marker_corners[2][0])])
 
                print("<<", marker_depth)
 
                if not np.isnan(marker_depth):
                    marker_distance = marker_depth
                    
                    if i == 3:
                        x0 = marker_center_x
                        y0 = marker_center_y
                        z0 = marker_depth
 
                    elif i == 4:
                        x1 = marker_center_x
                        y1 = marker_center_y
                        z1 = marker_depth
 
                else:
                    print("マーカーまでの距離がNaNです")
 
                    continue
 
                # z0かz1が0より大きく500mm以下ならプログラムおしまい
                if (z0>0 and z0 < 600) or (z1>0 and z1 < 600):
                    print("マーカーまでの距離が550mm以下なのでプログラムを終了します")
                    msg = OverrideRCIn()
                    msg.channels[servo_channel_forward - 1] = reverse_value
                    msg.channels[servo_channel_reverse - 1] = reverse_value
                    pub.publish(msg)
                    time.sleep(1)
                    
                    exit()
 
                cv2.circle(frame, (int(marker_center_x), int(marker_center_y)), 10, (0, 0, 255), -1)
 
                # マーカーのID番号を画面端に描画
                cv2.putText(frame, f"{i}", (int(marker_center_x), int(marker_center_y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
 
                # マーカーの4隅を描画
                cv2.polylines(frame, [marker_corners.astype(np.int32)], True, (0, 0, 255), 2, cv2.LINE_AA)
 
                # マーカーの中心座標を描画
                cv2.circle(frame, (int(marker_center[0]), int(marker_center[1])), 10, (0, 0, 255), -1)
 
                # # マーカーまでの距離を描画
                # cv2.putText(frame, f"{marker_distance:.2f} mm", (int(marker_center[0]), int(marker_center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
 
            if z0 == 0 or z1 == 0:
                print("3>>>>", x0, y0, z0)
                print("4>>>>", x1, y1, z1)
 
                # ちょっと前進する
                start_time = time.time()
                while time.time() - start_time < 0.2:
                    msg = OverrideRCIn()
                    msg.channels[servo_channel_forward - 1] = forward_value
                    msg.channels[servo_channel_reverse - 1] = reverse_value
                    pub.publish(msg)
                msg = OverrideRCIn()
                msg.channels[servo_channel_forward - 1] = reverse_value
                msg.channels[servo_channel_reverse - 1] = reverse_value
                pub.publish(msg)
                time.sleep(0.5)
 
                continue
 
        print("3:", (x0, y0, z0), "4:",(x1, y1, z1))
 
        M, P_up, P_down = find_point_P((x0, z0, 0), (x1, z1, 0), 200)
        cv2.circle(frame, (int(P_up[0]), int(y0)), 10, (0, 255, 0), -1)
        cv2.circle(frame, (int(P_down[0]), int(y0)), 10, (255, 0, 0), -1)
        print("Green:", P_up, "Blue:", P_down)
        M, P_up, P_down1 = find_point_P((x0, z0, 0), (x1, z1, 0), 550)
        
        # cv2.circle(frame, (int(P_up[0]), int(y0)), 10, (0, 255, 0), -1)
        cv2.circle(frame, (int(P_down1[0]), int(y0)), 10, (255, 255, 0), -1)
        print("Sky:", P_down1)
        capture_image(frame)
        
        # P_down1が右1/4にある時
        if P_down1[0] > frame.shape[1] / 4 * 3:
            turn_right(0.5)
            return 1
 
        # P_down1が左1/4にある時
        elif P_down1[0] < frame.shape[1] / 4:
            turn_left(0.5)
            return 1
 
        #P_down1が右1/4と中央の間にある時
        elif P_down1[0] < frame.shape[1] / 4 * 3 and P_down1[0] > frame.shape[1] / 8 * 5:
            turn_right(0.3)
            return 1
 
        # P_down1が左1/4と中央の間にある時
        elif P_down1[0] > frame.shape[1] / 4 and P_down1[0] < frame.shape[1] / 8 * 3:
            turn_left(0.3)
            return 1
 
        else:
            return 1
 
# 前進
def straight():
    msg = OverrideRCIn()
    
    # チャンネルを前進に設定
    msg.channels[servo_channel_forward] = forward_value
    msg.channels[servo_channel_reverse] = reverse_value
    pub.publish(msg)
    rate.sleep()

    return msg
 
# カメラ画像を取得
def get_camera():
    dat = camera.get_frame()
 
    if len(dat)==2:
        frame,depth=dat
 
    else:
        print('no image')
        return None,None
 
    corners, ids, rejectedImgPoints = detector.detectMarkers(frame)
    list_ids = np.ravel(ids)
    print("ids")
    print(list_ids)
 
    # cv2.imwrite("test_frame.jpg", frame)
    
    return frame,depth,corners,list_ids
 
 
# マーカーの中心座標を計算(マーカー1のみ)
def calc_marker_center(frame, corners, list_ids,id):
    marker_index = np.where(list_ids == id)[0][0]  # マーカー1のインデックスを取得
    marker_corners = corners[marker_index][0]
    
    # マーカーの中心座標を計算
    marker_center = np.mean(marker_corners, axis=0)
    marker_center_x = np.mean(marker_corners[:, 0])
    marker_center_y = np.mean(marker_corners[:, 1])

    return marker_center_x, marker_center_y
 
# 左回りで90度回転(マーカー1のみ)
def left_turn(msg, frame, depth, corners, list_ids, image_right_center_x, image_left_center_x, marker_center_x):
    marker_index = np.where(list_ids == 1)[0][0]  # マーカー1のインデックスを取得
    marker_corners = corners[marker_index][0]
    
    # マーカーの中心座標を計算
    marker_center = np.mean(marker_corners, axis=0)
    marker_center_x = np.mean(marker_corners[:, 0])
 
    # マーカーまでの距離を計算
    marker_depth = depth[int(marker_center[1]), int(marker_center[0])]

    if not np.isnan(marker_depth):
        marker_distance = marker_depth
        print(f"マーカー1までの距離: {marker_distance} mm")
        
        #距離が50cm未満なら曲がる
        if marker_distance < 800 and marker_distance != 0:
            print("50cm未満なので左旋回開始")
            
             # 左に90度回転
             # 時間
            # turn_left(2.0)
 
            # 壁を検出するまで(Happyでは1000) 
            detection_turn_left(1000)
            
            return 1
        
        # return 1
 
        #マーカーの中心座標が画面の右半分の半分にある場合は右に少し角度調整
        if marker_center_x > image_right_center_x :
            print("右に少し角度調整")
            
            #時間の調整が必要
            turn_right(0.5)
 
        if marker_center_x < image_left_center_x :
            print("左に少し角度調整")
 
            turn_left(0.5)
 
# 右回りで90度回転(マーカー2のみ)
 
def right_turn(msg, frame, depth, corners, list_ids, image_right_center_x, image_left_center_x, marker_center_x):
    marker_index = np.where(list_ids == 2)[0][0]  # マーカー2インデックスを取得
    marker_corners = corners[marker_index][0]
 
    # マーカーの中心座標を計算
    marker_center = np.mean(marker_corners, axis=0)
    marker_center_x = np.mean(marker_corners[:, 0])
 
    # マーカーまでの距離を計算
    marker_depth = depth[int(marker_center[1]), int(marker_center[0])]
 
    if not np.isnan(marker_depth):
        marker_distance = marker_depth
        print(f"マーカー2までの距離: {marker_distance} mm")
        print(marker_center_x)
        print(image_right_center_x)
        print(image_left_center_x)
 
        #距離が50cm未満なら曲がる
        if marker_distance < 800 and marker_distance != 0 :
            print("50cm未満なので右旋回開始")
            
            # 右に90度回転
            #　時間
            # turn_right(2.0)
            # 壁を検出するまで(Happyでは1000)
            detection_turn_right(1000)
 
            return 1
 
        # return 1
 
         #マーカーの中心座標が画面の右半分の半分にある場合は右に少し角度調整
 
        if marker_center_x > image_right_center_x :
            print("右に少し角度調整")
            turn_right(0.5)
 
        if marker_center_x < image_left_center_x :
            print("左に少し角度調整")
            turn_left(0.5)
 
# 停止
 
def marker_stop(msg, frame, depth, corners, list_ids, image_right_center_x, image_left_center_x, marker_center_x):
    marker_index = np.where(list_ids == 0)[0][0]  # マーカー1のインデックスを取得
    marker_corners = corners[marker_index][0]
 
    # マーカーの中心座標を計算
    marker_center = np.mean(marker_corners, axis=0)
 
    # マーカーの中心座標を画像中心に対するオフセットに変換
    image_center = np.array([frame.shape[1] / 2, frame.shape[0] / 2])
    offset = marker_center - image_center
 
    # マーカーまでの距離を計算
    marker_depth = depth[int(marker_center[1]), int(marker_center[0])]
 
    if not np.isnan(marker_depth):
        marker_distance = marker_depth
        print(f"マーカー0までの距離: {marker_distance} mm")

        # 距離が1m未満なら停止
        if marker_distance < 500 and marker_distance != 0:
            print("1m未満なので停止します")
            msg.channels[servo_channel_forward - 1] = reverse_value
            msg.channels[servo_channel_reverse - 1] = reverse_value
            pub.publish(msg)
 
            exit()
 
         #マーカーの中心座標が画面の右半分の半分にある場合は右に少し角度調整
        if marker_center_x > image_right_center_x :
            print("右に少し角度調整")
            
            #時間の調整が必要
            turn_right(0.5)
               
        elif marker_center_x < image_left_center_x :
            print("左に少し角度調整")
 
            #時間の調整が必要
            turn_left(0.5)
 
        else:
            #　関数使うのもあり(forword())
            msg = OverrideRCIn()
            msg.channels[servo_channel_forward - 1] = forward_value
            msg.channels[servo_channel_reverse - 1] = reverse_value
            pub.publish(msg)
            time.sleep(0.5)
 
 
def main():

    msg = None
    frame,depth,corners,list_ids = get_camera()
    start_time = time.time()
    image_left_center_x = frame.shape[1] / 4
    image_right_center_x = frame.shape[1] / 4 * 3
    i = 0
 
    while(1):
        i += 1
        #if time.time()-start_time > 0.5:
        if 1:
            #time.sleep(10)
            frame,depth,corners,list_ids = get_camera()
            start_time = time.time()
            msg = straight()
            capture_image(frame)
            
            if 1 in list_ids and len(corners) > 0:
                print("マーカー1が検出されました")
                print(f"{i}")
                
                # マーカの中心座標を計算
                marker_center_x, marker_center_y = calc_marker_center(frame, corners, list_ids,1)
                
                # left_turnの使用
                left_turn(msg, frame, depth, corners, list_ids, image_right_center_x, image_left_center_x, marker_center_x)
 
                time.sleep(1)
 
 
            if 2 in list_ids and len(corners) > 0:
                print("マーカー2が検出されました")
 
                # マーカの中心座標を計算
                marker_center_x, marker_center_y = calc_marker_center(frame, corners, list_ids,2)
 
                # right_turnの使用
                right_turn(msg, frame, depth, corners, list_ids, image_right_center_x, image_left_center_x, marker_center_x)
 
                time.sleep(3)
 
           
            # ステーション用
 
            if 3 in list_ids and len(corners) > 0:
                #マーカー4が見えるようにちょっと右に旋回
                # turn_right(0.5)
                print("マーカー3が検出されました")
                
                # マーカ４が見つからなかった時用にちょっと右に旋回
                if 4 not in list_ids:
                    print("マーカー4が見つからないので少し右に旋回します")
                    turn_right(0.5)
                    continue
                
                # station_detectionの使用
                station_detection(frame, depth, corners, list_ids)
 
                time.sleep(1)
 
                # 関数使ってもあり（forward())
                start_time = time.time()
                while time.time() - start_time < 0.3:
                    msg = OverrideRCIn()
                    msg.channels[servo_channel_forward - 1] = forward_value
                    msg.channels[servo_channel_reverse - 1] = reverse_value
                    pub.publish(msg)
                msg = OverrideRCIn()
                msg.channels[servo_channel_forward - 1] = reverse_value
                msg.channels[servo_channel_reverse - 1] = reverse_value
                pub.publish(msg)
                time.sleep(1)
 
            elif 4 in list_ids and len(corners) > 0:
                #マーカー3が見えるようにちょっと左に旋回
                print("マーカー4が検出されました")
                print("マーカー3が見えるようにちょっと左に旋回します")
                turn_left(0.5)
                time.sleep(1)
 
 
            if 0 in list_ids and len(corners) > 0:
                print("マーカー0が検出されました")
 
                # マーカの中心座標を計算
                marker_center_x, marker_center_y = calc_marker_center(frame, corners, list_ids,0)
 
                # stopの使用
                marker_stop(msg, frame, depth, corners, list_ids, image_right_center_x, image_left_center_x, marker_center_x)
 
 
            # マーカが見つからなかった時，壁を判別して真っ直ぐ進む
 
            if len(corners) == 0:
                print("マーカが見つからないので壁を判別して真っ直ぐ進みます")
 
                # 壁を判別して真っ直ぐ進む
                wall_detection(frame, depth)
                time.sleep(0.5)
 
        else:
            msg = straight()
 
if __name__ == '__main__':
 
    try:
        main()
        # While文を使って一秒に一回だけ関数を実行する
 
   
 
    except rospy.ROSInterruptException:
        pass
 
 