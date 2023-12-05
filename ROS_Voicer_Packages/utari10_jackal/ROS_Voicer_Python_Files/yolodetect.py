import yolov5
import cv2

class Yolo_Obj:
    c_x = 626
    c_y = 375
    label = False

    def obj_detect(image_ocv):
        # Perform object detection
        results = model(image_ocv)
        # Parse results
        predictions = results.pred[0]
        boxes = predictions[:, :4]  # x1, y1, x2, y2
        scores = predictions[:, 4]
        categories = predictions[:, 5]
        # Draw bounding boxes and labels on the frame
        for box, score, category in zip(boxes, scores, categories):
            x1, y1, x2, y2 = map(int, box)
            c_x = int((x1 + x2) / 2)
            c_y = int((y1 + y2) / 2)
            label = class_names[int(category)]
            color = class_colors[int(category)]
            cv2.circle(image_ocv, (c_x, c_y), 1, color, 2)
            cv2.rectangle(image_ocv, (x1, y1), (x2, y2), color, 2)
            cv2.putText(image_ocv, f'{label} {score:.6f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            print(f"CENTER of {label} is at {c_x,c_y}")

            if label == 'bottle':
                Yolo_Obj.c_x = c_x
                Yolo_Obj.c_y = c_y
                Yolo_Obj.label = True
                
            else:
                return
        pass 

model = yolov5.load('yolov5s.pt')
        # set model parameters
model.conf = 0.50  # NMS confidence threshold
model.iou = 0.45  # NMS IoU threshold
model.agnostic = False  # NMS class-agnostic
model.multi_label = False  # NMS multiple labels per box
model.max_det = 1000  # maximum number of detections per image
# Load class names
class_names = model.names
# Define color mapping based on YOLOv5's official color palette
color_palette = [
    (255, 0, 0),    # Red
    (0, 255, 0),    # Green
    (0, 0, 255),    # Blue
    (255, 255, 0),  # Yellow
    (0, 255, 255),  # Cyan
    (255, 0, 255)   # Magenta
]
class_colors = {i: color_palette[i % len(color_palette)] for i in range(len(class_names))}
