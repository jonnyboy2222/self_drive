from flask import Flask, Response, render_template_string, request, jsonify
import time
import requests
import threading
import pymysql
from dbutils.pooled_db import PooledDB
import json

# --- Configuration ---
REQUEST_TIMEOUT = 3.0

# ESP32 Communication URLs
ESP32_CONTROL_URL = "http://192.168.0.20:81/control" # ESP32's IP and control path

# --- RFID Signals ---
RFID_PASS_SIGNAL = "PASS" # RFID Tag Verified
RFID_FAIL_SIGNAL = "FAIL" # RFID Tag Not Verified


# --- Initialize Flask App ---
app = Flask(__name__)

# --- Global State for Command Sending ---
last_command_sent = None
command_lock = threading.Lock()
# Use a requests session for potential connection reuse
http_session = requests.Session()

# --- Database Connection Pool ---
try:
    db_pool = PooledDB(
        creator=pymysql,  # Module to use for creating connections
        maxconnections=5,  # Max number of connections in the pool
        mincached=2,     # Min number of idle connections to keep in cache
        host="localhost",
        user="root",
        password="kim4582345",
        database="johnbase",
        charset="utf8mb4",
        autocommit=True # Optional: set autocommit for connections from the pool
    )
    print("INFO: Database connection pool created successfully.")
except Exception as e:
    print(f"CRITICAL ERROR: Failed to create database connection pool: {e}")
    db_pool = None # Ensure db_pool is None if creation fails



def send_command_thread(cmd):
    """Sends command in a separate thread to avoid blocking."""
    global last_command_sent # This global might not be strictly necessary for RFID pass/fail signals
                             # if they are one-shot, but kept for consistency if other commands use it.

    try:
        params = {"cmd": cmd}
        # Use the session object
        res = http_session.get(ESP32_CONTROL_URL, params=params, timeout=REQUEST_TIMEOUT)
        res.raise_for_status() # Raise HTTPError for bad responses (4xx or 5xx)
        print(f"INFO: Command '{cmd}' sent successfully to {ESP32_CONTROL_URL}. Response: {res.text[:100]}")
        # For RFID pass/fail, updating last_command_sent might not be critical
        # as these are responses to specific events.
        # with command_lock:
        #     last_command_sent = cmd
    except requests.exceptions.Timeout:
        print(f"WARNING: Timeout sending command '{cmd}' to {ESP32_CONTROL_URL}")
        # with command_lock:
        #     last_command_sent = None
    except requests.exceptions.RequestException as e:
        print(f"ERROR: Error sending command '{cmd}' to {ESP32_CONTROL_URL}: {e}")
        # with command_lock:
        #     last_command_sent = None
    except Exception as e:
        print(f"ERROR: An unexpected error occurred in send_command_thread: {e}")



@app.route("/")
def index():
    """Serves the main HTML page with the video feed."""
    # Corrected HTML entities: &lt; becomes <, &gt; becomes >
    # Updated description for signals
    html_content = """
    <html>
    <head>
        <title>ESP32 Sensor Data Receiver & Control Server</title>
        {% raw %}
        <style>

            body {{ font-family: sans-serif; background-color: #f0f0f0; margin: 20px; }}
            h1 {{ color: #333; text-align: center; }}
            p {{ color: #555; text-align: center; }}
            .url {{ font-family: monospace; color: blue; background-color: #e0e0e0; padding: 2px 5px; border-radius: 3px; }}
            .info {{ max-width: 800px; margin: 0 auto; background-color: #fff; padding: 15px; border-radius: 5px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        </style>
        {% endraw %}
    </head>
    <body>
        <h1>ESP32 Sensor Data Receiver & Control Server</h1>
        <div class="info">
            <p>This server is running and can receive sensor data at the <span class="url">/sensor</span> endpoint.</p>
        </div>
    </body>
    </html>
    """

    return render_template_string(html_content)


def get_db_connection():
    """Gets a connection from the pool."""
    if not db_pool:
        raise Exception("Database pool is not initialized.")
    return db_pool.connection() # Get a connection from the pool



@app.route("/sensor", methods=["POST"])
def receive_sensor_data():
    try:
        data = request.get_json()
        if not data:
            return "Invalid JSON", 400

        sensor = data.get("sensor")
        time = data.get("time")
        value = data.get("data")

        if not all([sensor, time, value is not None]): # Check for presence of all three
            return "Missing data", 400

        # If 'value' is a list (like from GPS data), convert it to a JSON string
        # to store in a single database column.
        if isinstance(value, list):
            value_to_store = json.dumps(value)
        else:
            value_to_store = value


        # Use context managers for connection and cursor
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                sql = "INSERT INTO test (sensor, realtime, data) VALUES (%s, %s, %s)"
                cursor.execute(sql, (sensor, time, value_to_store))
        
        # Connection and cursor are automatically closed/returned to pool here   

        return "Data stored successfully", 200
    
    except pymysql.Error as db_err:
        print(f"DATABASE ERROR in /sensor: {db_err}")
        return f"Database Error: {str(db_err)}", 500
    except Exception as e:
        import traceback
        print(f"UNEXPECTED ERROR in /sensor: {str(e)}")
        traceback.print_exc() # This will print the full stack trace
        return f"An unexpected error occurred: {str(e)}", 500

    

@app.route("/rfid_check", methods=["POST"])
def rfid_check():
    """Receives RFID tag, verifies against DB, sends pass/fail signal to ESP32."""
    try:
        data = request.get_json()
        if not data:
            return jsonify({"status": "error", "message": "Invalid JSON"}), 400

        rfid_tag = data.get("rfid_tag")
        if not rfid_tag:
            return jsonify({"status": "error", "message": "Missing 'rfid_tag' in JSON"}), 400

        print(f"INFO: Received RFID check request for tag: {rfid_tag}")

        
        rfid_verified = False
        # Use context managers for connection and cursor
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                # Ensure you have a table like 'authorized_rfids' with a 'tag_id' column
                sql = "SELECT COUNT(*) FROM authorized_rfids WHERE tag_id = %s"
                cursor.execute(sql, (rfid_tag,))
                result = cursor.fetchone()
                if result and result[0] > 0:
                    rfid_verified = True
        # Connection and cursor are automatically closed/returned to pool here

        if rfid_verified:
            print(f"INFO: RFID tag '{rfid_tag}' VERIFIED. Sending PASS signal.")
            threading.Thread(target=send_command_thread, args=(RFID_PASS_SIGNAL,), daemon=True).start()
            return jsonify({"status": "success", "message": "RFID verified", "result": "pass"}), 200
        else:
            print(f"INFO: RFID tag '{rfid_tag}' NOT VERIFIED. Sending FAIL signal.")
            threading.Thread(target=send_command_thread, args=(RFID_FAIL_SIGNAL,), daemon=True).start()
            return jsonify({"status": "success", "message": "RFID not verified", "result": "fail"}), 200

    except pymysql.Error as db_err:
        print(f"DATABASE ERROR in /rfid_check: {db_err}")
        return jsonify({"status": "error", "message": f"Database Error: {str(db_err)}"}), 500
    except Exception as e:
        import traceback
        print(f"UNEXPECTED ERROR in /rfid_check: {str(e)}")
        traceback.print_exc() # This will print the full stack trace
        return jsonify({"status": "error", "message": f"An unexpected error occurred: {str(e)}"}), 500



def cleanup():
    """Release resources."""
    print("\nShutting down... Releasing resources.")
    
    # Close the requests session
    if http_session:
        http_session.close()
        print("INFO: HTTP session closed.")

    print("INFO: Cleanup finished.")


if __name__ == "__main__":
    print("INFO: Starting Flask server...")
    print(f"INFO: Will send commands to ESP32 Control: {ESP32_CONTROL_URL}")
    try:
        if not db_pool:
            print("WARNING: Database pool not available. /sensor endpoint might not work.")
        # Use threaded=True for handling multiple clients/requests
        # Use debug=False for production/streaming stability
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
         print("INFO: Keyboard interrupt received. Exiting.")
    except Exception as e:
         print(f"ERROR: Flask server failed: {e}")
    finally:
        cleanup()