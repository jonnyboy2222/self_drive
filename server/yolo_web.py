from flask import Flask, Response, render_template_string
import cv2
import time
from ultralytics import YOLO
import torch
import requests
import threading
import numpy as np

# --- Configuration ---
YOLO_MODEL_PATH = "yolov8s.pt"
TARGET_CLASS_PERSON = "person" # Renamed for clarity
TARGET_CLASS_TRAFFIC_LIGHT = "traffic light"
CONFIDENCE_THRESHOLD = 0.5
REQUEST_TIMEOUT = 1.0

# ESP32 Communication URLs
ESP32_CAM_STREAM_URL = "http://192.168.0.187:81/stream" # variable
ESP32_CONTROL_URL = "http://192.168.0.187:81/control"

# --- Signals ---
# Define distinct signals for different states (adjust characters as needed)
PERSON_DETECTED_SIGNAL = "P"  # Person detected - Highest priority
RED_LIGHT_SIGNAL = "R"      # Red light detected
ORANGE_LIGHT_SIGNAL = "O"   # Orange light detected
GREEN_LIGHT_SIGNAL = "G"    # Green light detected
NO_PERSON_NO_LIGHT_SIGNAL = "N" # Default state (no person, no significant light)

# --- HSV Color Ranges ---
RED_RANGES = [
    {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
    {'lower': np.array([160, 100, 50]), 'upper': np.array([179, 255, 255])}
]
GREEN_RANGE = {'lower': np.array([40, 50, 50]), 'upper': np.array([85, 255, 255])}
ORANGE_RANGE = {'lower': np.array([10, 100, 100]), 'upper': np.array([25, 255, 255])}
MIN_PIXEL_PERCENT_THRESHOLD = 0.05 # 5% of ROI area needed to confirm color

# --- Drawing Colors (BGR) ---
PERSON_COLOR = (0, 255, 0)
RED_LIGHT_COLOR = (0, 0, 255)
GREEN_LIGHT_COLOR = (0, 255, 0)
ORANGE_LIGHT_COLOR = (0, 165, 255) # Orange/Amber
UNKNOWN_LIGHT_COLOR = (0, 255, 255) # Yellow

# --- Initialize Flask App ---
app = Flask(__name__)

# --- Initialize YOLO Model ---
print(f"Loading YOLOv8 model from {YOLO_MODEL_PATH}...")
model = None
try:
    model = YOLO(YOLO_MODEL_PATH)
    if torch.cuda.is_available():
        print("CUDA (GPU) available, using GPU for YOLO.")
        model.to("cuda")
    else:
        print("CUDA not available, using CPU for YOLO.")
        model.to("cpu")
    print("YOLOv8 model loaded successfully.")
except Exception as e:
    print(f"ERROR: Error loading YOLO model: {e}")
    # Decide if you want to exit or continue without model
    # exit()

# --- Initialize Camera ---
print(f"Attempting to open camera stream at {ESP32_CAM_STREAM_URL}...")
camera = cv2.VideoCapture(ESP32_CAM_STREAM_URL)

if not camera.isOpened():
    print(f"CRITICAL ERROR: Could not open camera stream at {ESP32_CAM_STREAM_URL}.")
    exit()
else:
    print("Camera stream opened successfully.")

# --- Global State for Command Sending ---
last_command_sent = None
# lock is like a token, only one thread can modify data
command_lock = threading.Lock()
# to keep http connection alive (reuse) instead of creating new in a row
http_session = requests.Session()

def send_command_thread(cmd):
    """Sends command in a separate thread to avoid blocking."""
    global last_command_sent

    try:
        # ex) http://<ESP_IP>:81/control?cmd=P
        params = {"cmd": cmd}
        # Use the session object
        res = http_session.get(ESP32_CONTROL_URL, params=params, timeout=REQUEST_TIMEOUT)
        res.raise_for_status() # Raise HTTPError for bad responses (4xx or 5xx)
        print(f"INFO: Command '{cmd}' sent successfully to {ESP32_CONTROL_URL}. Response: {res.text[:100]}")
        # Update last command sent only on success
        with command_lock:
            last_command_sent = cmd
    except requests.exceptions.Timeout:
        print(f"WARNING: Timeout sending command '{cmd}' to {ESP32_CONTROL_URL}")
        with command_lock:
            last_command_sent = None # Allow retry on next state change
    except requests.exceptions.RequestException as e:
        print(f"ERROR: Error sending command '{cmd}' to {ESP32_CONTROL_URL}: {e}")
        with command_lock:
            last_command_sent = None
    except Exception as e:
        print(f"ERROR: An unexpected error occurred in send_command_thread: {e}")
        with command_lock:
            last_command_sent = None


def detect_traffic_light_color(roi):
    """Analyzes ROI to determine traffic light color."""
    if roi.size == 0:
        print("WARNING: Traffic light ROI is empty.")
        return "UNKNOWN", UNKNOWN_LIGHT_COLOR

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    roi_area = roi.shape[0] * roi.shape[1]
    min_pixels_threshold = roi_area * MIN_PIXEL_PERCENT_THRESHOLD

    # Calculate pixel counts for each color
    red_pixels = 0
    for r in RED_RANGES:
        mask = cv2.inRange(hsv_roi, r["lower"], r["upper"])
        red_pixels += cv2.countNonZero(mask)

    green_mask = cv2.inRange(hsv_roi, GREEN_RANGE["lower"], GREEN_RANGE["upper"])
    green_pixels = cv2.countNonZero(green_mask)

    orange_mask = cv2.inRange(hsv_roi, ORANGE_RANGE["lower"], ORANGE_RANGE["upper"])
    orange_pixels = cv2.countNonZero(orange_mask)

    # Determine the dominant color based on pixel count and threshold
    light_color = "UNKNOWN"
    draw_color = UNKNOWN_LIGHT_COLOR

    # Simple priority: Red > Green > ORANGE (adjust if needed)
    # Corrected HTML entities: > becomes >
    if red_pixels > min_pixels_threshold and red_pixels >= green_pixels and red_pixels >= orange_pixels:
        light_color = "RED"
        draw_color = RED_LIGHT_COLOR
    elif green_pixels > min_pixels_threshold and green_pixels > red_pixels and green_pixels >= orange_pixels:
        light_color = "GREEN"
        draw_color = GREEN_LIGHT_COLOR
    elif orange_pixels > min_pixels_threshold and orange_pixels > red_pixels and orange_pixels > green_pixels:
        light_color = "ORANGE"
        draw_color = ORANGE_LIGHT_COLOR

    return light_color, draw_color


# --- Add near the top Configuration section ---
FRAME_SKIP = 3 # Process only every 3rd frame (Adjust this value!)

# ... other code ...

def generate_frames():
    """Generator function: captures frames, runs YOLO periodically, draws boxes, yields frame, sends commands."""
    global last_command_sent

    frame_count = 0
    # Store results from the last processed frame to use for skipped frames
    last_detection_state = {
        "person_detected": False,
        "red_light_detected": False,
        "green_light_detected": False,
        "orange_light_detected": False,
        "items_for_drawing": []
    }
    # Store the actual annotated image from the last processed frame
    last_processed_image = None

    while True:
        if not camera.isOpened():
            print("ERROR: Camera is not open in generate_frames loop. Retrying...")
            time.sleep(max(REQUEST_TIMEOUT, 1.0))
            continue

        success, frame = camera.read()
        if not success or frame is None:
            print("WARNING: Failed to grab frame. Retrying...")
            time.sleep(0.1)
            continue

        frame_count += 1
        display_frame = frame.copy() # Frame to be encoded and sent

        # --- Process frame only periodically ---
        if frame_count % FRAME_SKIP == 0:
            # Reset state for this processing cycle
            person_detected_in_frame = False
            red_light_detected = False
            green_light_detected = False
            orange_light_detected = False
            detected_items_for_drawing = []

            # --- Run YOLO Detection ---
            if model:
                try:
                    results = model(frame, stream=False, verbose=False, conf=CONFIDENCE_THRESHOLD)
                    for result in results:
                        boxes = result.boxes
                        for box in boxes:
                            try:
                                conf = float(box.conf[0])
                                cls = int(box.cls[0])
                                class_name = model.names[cls]
                                x1, y1, x2, y2 = map(int, box.xyxy[0])

                                if class_name.lower() == TARGET_CLASS_PERSON.lower():
                                    person_detected_in_frame = True
                                    label = f"{class_name}: {conf:.2f}"
                                    detected_items_for_drawing.append({
                                        "box": (x1, y1, x2, y2), "label": label, "color": PERSON_COLOR
                                    })
                                elif class_name.lower() == TARGET_CLASS_TRAFFIC_LIGHT.lower():
                                    roi = frame[y1:y2, x1:x2]
                                    light_color, draw_color = detect_traffic_light_color(roi)
                                    if light_color == "RED": 
                                        red_light_detected = True
                                    elif light_color == "GREEN": 
                                        green_light_detected = True
                                    elif light_color == "ORANGE": 
                                        orange_light_detected = True
                                    label = f"Light: {light_color} ({conf:.2f})"
                                    detected_items_for_drawing.append({
                                        "box": (x1, y1, x2, y2), "label": label, "color": draw_color
                                    })
                            except Exception as e:
                                print(f"ERROR: Error processing a detection box (Class ID: {cls}, Conf: {conf:.2f}): {e}")
                                continue
                except Exception as e:
                    print(f"ERROR: Error during YOLO inference: {e}")

            # --- Update last known state ---
            last_detection_state["person_detected"] = person_detected_in_frame
            last_detection_state["red_light_detected"] = red_light_detected
            last_detection_state["green_light_detected"] = green_light_detected
            last_detection_state["orange_light_detected"] = orange_light_detected
            last_detection_state["items_for_drawing"] = detected_items_for_drawing

            # --- Draw on the current frame ---
            for item in detected_items_for_drawing:
                x1, y1, x2, y2 = item["box"]
                label = item["label"]
                color = item["color"]
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                label_y = y1 - 10 if y1 - 10 > 10 else y1 + 20
                cv2.putText(display_frame, label, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            last_processed_image = display_frame.copy() # Save this annotated image

        # --- For skipped frames, reuse last annotations ---
        elif last_processed_image is not None:
             # Option 1: Display the *last processed* image directly (can look jerky)
             # display_frame = last_processed_image

             # Option 2: Draw *old* boxes on the *new* frame (smoother motion, potentially inaccurate boxes)
             for item in last_detection_state["items_for_drawing"]:
                 x1, y1, x2, y2 = item["box"]
                 label = item["label"]
                 color = item["color"]
                 cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                 label_y = y1 - 10 if y1 - 10 > 10 else y1 + 20
                 cv2.putText(display_frame, label, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


        # --- Determine Command Based on Priority (using the *last known state*) ---
        current_command = NO_PERSON_NO_LIGHT_SIGNAL
        action_text = f"State: Default ({NO_PERSON_NO_LIGHT_SIGNAL})"

        if last_detection_state["person_detected"]:
            current_command = PERSON_DETECTED_SIGNAL
            action_text = f"State: Person ({PERSON_DETECTED_SIGNAL})"
        elif last_detection_state["red_light_detected"]:
            current_command = RED_LIGHT_SIGNAL
            action_text = f"State: Red Light ({RED_LIGHT_SIGNAL})"
        elif last_detection_state["orange_light_detected"]:
            current_command = ORANGE_LIGHT_SIGNAL
            action_text = f"State: Orange Light ({ORANGE_LIGHT_SIGNAL})"
        elif last_detection_state["green_light_detected"]:
            current_command = GREEN_LIGHT_SIGNAL
            action_text = f"State: Green Light ({GREEN_LIGHT_SIGNAL})"

        # Display the determined action on the frame being sent
        cv2.putText(display_frame, action_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        # --- Send Command if State Changed ---
        with command_lock:
            should_send = (current_command != last_command_sent)
        if should_send:
            print(f"INFO: {action_text}. Sending command '{current_command}'...")
            thread = threading.Thread(target=send_command_thread, args=(current_command,), daemon=True)
            thread.start()

        # --- Encode and Yield Frame ---
        try:
            ret, buffer = cv2.imencode(".jpg", display_frame) # Encode the frame with annotations
            if not ret:
                print("WARNING: Failed to encode frame.")
                continue
            frame_bytes = buffer.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")
        except Exception as e:
            print(f"ERROR: Error encoding or yielding frame: {e}")


@app.route("/")
def index():
    """Serves the main HTML page with the video feed."""
    # Corrected HTML entities: &lt; becomes <, &gt; becomes >
    # Updated description for signals
    html_content = """
    <html>
    <head>
        <title>YOLOv8 Object Detection Stream</title>
        <style>
            body {{ font-family: sans-serif; background-color: #f0f0f0; margin: 20px; }}
            h1 {{ color: #333; text-align: center; }}
            p {{ color: #555; text-align: center; }}
            img {{ display: block; margin: 20px auto; border: 2px solid black; background-color: #fff; max-width: 90%; height: auto; }}
            .url {{ font-family: monospace; color: blue; background-color: #e0e0e0; padding: 2px 5px; border-radius: 3px; }}
            .info {{ max-width: 800px; margin: 0 auto; background-color: #fff; padding: 15px; border-radius: 5px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        </style>
    </head>
    <body>
        <h1>Live ESP32 Camera Feed with YOLOv8 Detection</h1>
        <div class="info">
            <p>Detecting: '{}' and '{}' (Confidence > {})</p>
            <p>Sending commands ('{}', '{}', '{}', '{}', '{}') based on priority (Person > Red > Orange > Green > None) to ESP32 at: <br><span class="url">{}</span></p>
        </div>
        <img src="{}" alt="Live video feed">
    </body>
    </html>
    """.format(
        TARGET_CLASS_PERSON.capitalize(),
        TARGET_CLASS_TRAFFIC_LIGHT.capitalize(),
        CONFIDENCE_THRESHOLD,
        PERSON_DETECTED_SIGNAL,
        RED_LIGHT_SIGNAL,
        ORANGE_LIGHT_SIGNAL,
        GREEN_LIGHT_SIGNAL,
        NO_PERSON_NO_LIGHT_SIGNAL,
        ESP32_CONTROL_URL, # Show the control URL
        '{{ url_for("video_feed") }}' # Flask's way to generate URL
        )
    return render_template_string(html_content)

@app.route("/video_feed")
def video_feed():
    """Route that streams video frames."""
    if not camera or not camera.isOpened():
        print("ERROR: Video feed request failed: Camera not available.")
        return "Error: Camera not available.", 503 # Service Unavailable
    if not model:
        print("ERROR: Video feed request failed: YOLO model not loaded.")
        return "Error: YOLO model not loaded.", 503 # Service Unavailable

    print("INFO: Client connected to video feed.")
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

def cleanup():
    """Release resources."""
    print("\nShutting down... Releasing resources.")
    if camera and camera.isOpened():
        camera.release()
        print("INFO: Camera released.")
    # Close the requests session
    http_session.close()
    print("INFO: HTTP session closed.")
    cv2.destroyAllWindows()
    print("INFO: Cleanup finished.")

if __name__ == "__main__":
    print("INFO: Starting Flask server...")
    # Corrected HTML entity: &lt; becomes <
    print(f"INFO: Access the stream at http://127.0.0.1:5000/ or http://<your-ip-address>:5000/")
    print(f"INFO: Using ESP32 Stream: {ESP32_CAM_STREAM_URL}")
    print(f"INFO: Sending commands to ESP32 Control: {ESP32_CONTROL_URL}")
    try:
        # Use threaded=True for handling multiple clients/requests
        # Use debug=False for production/streaming stability
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
         print("INFO: Keyboard interrupt received. Exiting.")
    except Exception as e:
         print(f"ERROR: Flask server failed: {e}")
    finally:
        cleanup()