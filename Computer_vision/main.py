from ultralytics import YOLO
import cv2
import supervision as sv
from search_yourtube import search_yt, display_yt_results

# Initialize a YOLO-World model
#model = YOLO("yolov8s-world.pt")  # or choose yolov8m/l-world.pt
#model = YOLO("yolov8m-world.pt")
model = YOLO("yolov8l-world.pt")

# Define custom classes
model.set_classes(["lamp", "pen", "phone", "superman", "giraffe"])

bounding_box_annotator = sv.BoundingBoxAnnotator()
lable_annotator = sv.LabelAnnotator()

cap = cv2.VideoCapture(1)
w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))

out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), fps, (w, h))

class_labels = {0: "lamp", 1: "pen", 2: "phone", 3: "superman", 4: "giraffe"}

while cap.isOpened():
    take_screenshot = input("Press enter to take a screenshot, or 'q' to quit: ")
    if take_screenshot == "q":
        break
    elif take_screenshot == "":
        ret, img = cap.read()
        if not ret:
            break

        # Execute prediction for specified categories on an image
        results = model.predict(img)

        detections = sv.Detections.from_ultralytics(results[0])

        # Check for 'lamp' in detections and print confidence
        for i, class_id in enumerate(detections.class_id):
            if class_labels[class_id] == "lamp":
                lamp_confidence = detections.confidence[i]
                print(f"Lamp detected with confidence: {lamp_confidence * 100:.2f}%\n\n\n")
                search_response = search_yt('lamp')
                display_yt_results(search_response)
            elif class_labels[class_id] == "pen":
                pen_confidence = detections.confidence[i]
                print(f"Pen detected with confidence: {pen_confidence * 100:.2f}%\n\n\n")
                search_response = search_yt('pen')
                display_yt_results(search_response)
            elif class_labels[class_id] == "phone":
                phone_confidence = detections.confidence[i]
                print(f"Phone detected with confidence: {phone_confidence * 100:.2f}%\n\n\n")
                search_response = search_yt('phone')
                display_yt_results(search_response)
            elif class_labels[class_id] == "superman":
                superman_confidence = detections.confidence[i]
                print(f"Superman detected with confidence: {superman_confidence * 100:.2f}%\n\n\n")
                search_response = search_yt('superman')
                display_yt_results(search_response)
            elif class_labels[class_id] == "giraffe":
                giraffe_confidence = detections.confidence[i]
                print(f"Giraffe detected with confidence: {giraffe_confidence * 100:.2f}%\n\n\n")
                search_response = search_yt('giraffe')
                display_yt_results(search_response)
            else:
                print("No object detected")
                search_response = search_yt('no object detected')
                display_yt_results(search_response)

        annotated_frame = bounding_box_annotator.annotate(
            scene=img.copy(),
            detections=detections
        )

        annotated_frame = lable_annotator.annotate(
            scene=annotated_frame,
            detections=detections
        )

        out.write(annotated_frame)
        cv2.imshow("Image", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()
