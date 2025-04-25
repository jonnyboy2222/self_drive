from flask import Flask, Response, render_template_string
import cv2
import time # Optional: for adding delays or frame rate control

app = Flask(__name__)

# --- Camera Setup ---
CAMERA_INDEX = 0
ESP32_CAM_STREAM_URL = "http://192.168.2.28:81/stream" # variable

print("Waiting briefly before opening camera...")
time.sleep(1)
print("Attempting to open laptop camera...")
laptop_camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

print("Waiting briefly before opening camera...")
time.sleep(1)
print("Attempting to open esp32 camera...")
esp32_camera = cv2.VideoCapture(ESP32_CAM_STREAM_URL)

# Check if the camera opened successfully
if not laptop_camera.isOpened():
    print(f"Error: Could not open camera at index {CAMERA_INDEX}.")

else:
    print("Laptop Camera opened successfully.")

# Check if the camera opened successfully
if not esp32_camera.isOpened():
    print(f"Error: Could not open camera at index {ESP32_CAM_STREAM_URL}.")

else:
    print("ESP32 Camera opened successfully.")
    
def laptop_generate_frames():
    """Generator function to capture frames from the camera and yield them."""
    while True:
        # Read a frame from the camera
        success, frame = laptop_camera.read()
        if not success:
            print("LAPTOP: Failed to grab frame. Retrying...")
            time.sleep(0.1)
            continue
        else:
            # Encode the frame as JPEG
            # JPEG is efficient for streaming over the web
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                print("LAPTOP: Failed to encode frame.")
                continue # Skip this frame

            # Convert the buffer to bytes
            frame_bytes = buffer.tobytes()

            # Yield the frame in the multipart format
            # The boundary 'frame' is defined in the Response mimetype
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        # Optional: Control frame rate (e.g., limit to ~30 FPS)
        # time.sleep(0.03)

def esp32_generate_frames():
    """Generator function to capture frames from the camera and yield them."""
    while True:
        # Read a frame from the camera
        success, frame = esp32_camera.read()
        if not success:
            print("ESP32: Failed to grab frame. Retrying...")
            time.sleep(0.1)
            continue
        else:
            # --- 필터 추가 시작 ---
            denoised = cv2.GaussianBlur(frame, (5, 5), 0)
            sharpened = cv2.addWeighted(frame, 1.5, denoised, -0.5, 0)
            # --- 필터 추가 끝 ---

            ret, buffer = cv2.imencode('.jpg', sharpened)
            if not ret:
                print("ESP32: Failed to encode frame.")
                continue # Skip this frame

            # Convert the buffer to bytes
            frame_bytes = buffer.tobytes()

            # Yield the frame in the multipart format
            # The boundary 'frame' is defined in the Response mimetype
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        # Optional: Control frame rate (e.g., limit to ~30 FPS)
        # time.sleep(0.03)


@app.route('/')
def index():
    """Serves the main HTML page with both video feeds."""
    # HTML page with two img tags pointing to the respective video feed routes
    html_content = """
    <html>
    <head>
        <title>Multi-Camera Video Stream</title>
        {# Use raw block to prevent Jinja2 from parsing CSS braces #}
        {% raw %}
        <style>
            body { display: flex; justify-content: space-around; align-items: flex-start; }
            .feed-container { border: 1px solid #ccc; padding: 10px; margin: 10px; background-color: #f9f9f9;}
            h1, h2 { text-align: center; }
            img { display: block; margin-top: 5px; background-color: #eee; }
        </style>
        {% endraw %}
        {# End of raw block #}
    </head>
    <body>
        <h1>Live Camera Feeds</h1>
        <div class="feed-container">
            <h2>Built-in Camera</h2>
            {# Use url_for to generate the correct URL for the route function #}
            <img src="{{ url_for('builtin_video_feed') }}" width="640" height="480" alt="Built-in Camera Feed">
        </div>
        <div class="feed-container">
            <h2>ESP32-CAM</h2>
            {# Use url_for to generate the correct URL for the route function #}
            <img src="{{ url_for('esp_video_feed') }}" width="640" height="480" alt="ESP32-CAM Feed">
        </div>
    </body>
    </html>
    """
    return render_template_string(html_content)


@app.route('/laptop_video_feed')
def builtin_video_feed():
    """Route that streams video frames from the built-in camera."""
    if not laptop_camera or not laptop_camera.isOpened():
        return "Error: Built-in Camera not available.", 503 # Service Unavailable

    return Response(laptop_generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/esp32_video_feed')
def esp_video_feed():
    """Route that streams video frames from the ESP32-CAM."""
    if not esp32_camera or not esp32_camera.isOpened():
        return "Error: ESP32-CAM stream not available.", 503 # Service Unavailable

    return Response(esp32_generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def cleanup():
    """Release camera resources."""
    print("\nShutting down... Releasing resources.")
    if laptop_camera and laptop_camera.isOpened():
        laptop_camera.release()
        print("Built-in camera released.")
    if esp32_camera and esp32_camera.isOpened():
        esp32_camera.release()
        print("ESP32-CAM stream released.")
    cv2.destroyAllWindows()
    print("Cleanup finished.")

if __name__ == '__main__':
    print("Starting Flask server...")
    try:
        # threaded=True allows Flask to handle multiple stream requests concurrently
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
         print("Keyboard interrupt received.")
    finally:
        # Ensure cleanup runs when the server stops
        cleanup()