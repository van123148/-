import cv2
import numpy as np
import onnxruntime as ort
from ultralytics import YOLO

# 相机内参
camera_matrix = np.array([
    [2075.15666, 0., 646.02307],
    [0., 2073.92438, 479.8963],
    [0., 0., 1.]
])
dist_coeffs = np.array([-0.051148, 0.231678, 0.000775, 0.002697, 0.000000])

# 3D参考点
buff_3d_points = np.array([
    [0, 0.1700, 0.1750],
    [0, -0.1700, 0.1750],
    [0, -0.1850, -0.1650],
    [0, 0, -0.7150],
    [0, 0.1850, -0.1650]
], dtype=np.float32)


def nms(boxes, scores, iou_threshold):
    sorted_indices = np.argsort(scores)[::-1]
    keep_boxes = []
    while sorted_indices.size > 0:
        box_id = sorted_indices[0]
        keep_boxes.append(box_id)
        ious = compute_iou(boxes[box_id, :], boxes[sorted_indices[1:], :])
        keep_indices = np.where(ious < iou_threshold)[0]
        sorted_indices = sorted_indices[keep_indices + 1]
    return keep_boxes


def compute_iou(box, boxes):
    inter_area = np.maximum(0, np.minimum(box[2], boxes[:, 2]) - np.maximum(box[0], boxes[:, 0])) * \
                 np.maximum(0, np.minimum(box[3], boxes[:, 3]) - np.maximum(box[1], boxes[:, 1]))
    box_area = (box[2] - box[0]) * (box[3] - box[1])
    boxes_area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    iou = inter_area / (box_area + boxes_area - inter_area)
    return iou


def solve_pnp(image_points):
    if len(image_points) != 5:
        return None, None

    success, rotation_vector, translation_vector = cv2.solvePnP(
        buff_3d_points, image_points, camera_matrix, dist_coeffs)

    if success:
        return rotation_vector, translation_vector
    else:
        return None, None


# 加载模型
model = YOLO(r'best_csf.onnx', task='pose')

video_path = 'test_video.mp4'
cap = cv2.VideoCapture(video_path)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

output_path = 'output_video.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

frame_count = 0


def safe_int_tuple(point):
    try:
        x, y = point.ravel()
        if not (np.isfinite(x) and np.isfinite(y)):
            return None
        if abs(x) > 1e6 or abs(y) > 1e6:  # 添加一个合理的阈值
            return None
        return (int(x), int(y))
    except:
        print(f"Error converting point to int tuple: {point}")
        return None


while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # 使用模型进行预测
    results = model(frame, conf=0.3, iou=0.5,device='cpu')

    detections = []
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()

        # 应用NMS
        keep_boxes = nms(boxes, scores, iou_threshold=0.5)

        for idx in keep_boxes:
            x1, y1, x2, y2 = boxes[idx]
            score = scores[idx]
            class_id = class_ids[idx]
            detections.append((x1, y1, x2, y2, score, class_id))

    # 提取中心点
    center_points = []
    for det in detections:
        x1, y1, x2, y2, _, class_id = det
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        center_points.append((center_x, center_y, class_id))

    print(f"Detected {len(center_points)} points")

    # 如果检测到5个点，进行PnP求解
    if len(center_points) == 5:
        image_points = np.array([(x, y) for x, y, _ in center_points], dtype=np.float32)
        rotation_vector, translation_vector = solve_pnp(image_points)

        if rotation_vector is not None and translation_vector is not None:
            # 在图像上绘制坐标轴
            axis_length = 0.1
            axis_points, _ = cv2.projectPoints(
                np.array([(0, 0, 0), (axis_length, 0, 0), (0, axis_length, 0), (0, 0, axis_length)]),
                rotation_vector, translation_vector, camera_matrix, dist_coeffs)

            print(f"Axis points shape: {axis_points.shape}")
            print(f"Axis points: {axis_points}")

            # 确保所有点都是整数元组
            origin = safe_int_tuple(axis_points[0])
            point_x = safe_int_tuple(axis_points[1])
            point_y = safe_int_tuple(axis_points[2])
            point_z = safe_int_tuple(axis_points[3])

            if all([origin, point_x, point_y, point_z]):
                cv2.line(frame, origin, point_x, (0, 0, 255), 2)  # X轴 - 红色
                cv2.line(frame, origin, point_y, (0, 255, 0), 2)  # Y轴 - 绿色
                cv2.line(frame, origin, point_z, (255, 0, 0), 2)  # Z轴 - 蓝色

                # 在图像上显示旋转向量和平移向量
                cv2.putText(frame, f"R: {rotation_vector.ravel()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0), 2)
                cv2.putText(frame, f"T: {translation_vector.ravel()}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0), 2)
            else:
                print("Error: Unable to draw coordinate axes due to invalid points")
    else:
        print(f"Not enough points detected for PnP solve: {len(center_points)}")

    # 在图像上绘制检测结果
    for det in detections:
        x1, y1, x2, y2, score, class_id = det
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"Class: {int(class_id)}, Score: {score:.2f}", (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 显示帧计数
    cv2.putText(frame, f"Frame: {frame_count}", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # 将处理后的帧写入输出视频文件
    out.write(frame)

# 释放视频捕获和写入对象
cap.release()
out.release()
