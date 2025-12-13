from flask import Flask, render_template, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
import threading
import random
import time
import os
import math
import json

with open('config.json', 'r') as f:
    config = json.load(f)

os.makedirs('templates', exist_ok=True)

app = Flask(__name__, template_folder='templates')
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
socketio = SocketIO(app, cors_allowed_origins="*")

# Track center coordinates
TRACK_CENTER_LAT = 46.425685
TRACK_CENTER_LON = 25.390393
TRACK_RADIUS = 0.0003

# Generate circular test path
def generate_track_path():
    """Generate a circular path around the track"""
    path = []
    points = 100
    for i in range(points):
        angle = (i / points) * 2 * math.pi
        lat = TRACK_CENTER_LAT + (TRACK_RADIUS * math.cos(angle))
        lon = TRACK_CENTER_LON + (TRACK_RADIUS * math.sin(angle))
        path.append((lat, lon))
    return path

track_path = generate_track_path()
current_position_index = 0

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing between two points in degrees"""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon_rad = math.radians(lon2 - lon1)
    
    y = math.sin(dlon_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

# Simulate telemetry data
def generate_telemetry():
    """Generate realistic Formula Student telemetry data with GPS"""
    global current_position_index
    
    baseline_time = time.time()
    speed = 0
    throttle = 0
    
    while True:
        # Get current position from track path
        lat, lon = track_path[current_position_index]
        
        # Calculate bearing to next point
        next_index = (current_position_index + 1) % len(track_path)
        next_lat, next_lon = track_path[next_index]
        heading = calculate_bearing(lat, lon, next_lat, next_lon)
        
        # Move to next position
        current_position_index = next_index
        
        # Simulate acceleration/deceleration
        speed += random.uniform(-5, 8)
        speed = max(0, min(230, speed))  # Clamp 0-230 km/h
        
        throttle += random.uniform(-10, 15)
        throttle = max(0, min(100, throttle))  # Clamp 0-100%
        
        telemetry = {
            'timestamp': int((time.time() - baseline_time) * 1000),  # ms
            # GPS data
            'latitude': lat,
            'longitude': lon,
            'heading': heading,  # bearing in degrees (0-360)
            'altitude': random.uniform(620, 625),  # meters
            # Engine data
            'agt': random.uniform(20, 60),           # Air intake temp (°C)
            'egt': random.uniform(400, 900),         # Exhaust temp (°C)
            'lambda': random.uniform(0.98, 1.02),    # Lambda ratio
            'feszultseg': random.uniform(12, 14.5),  # Voltage (V)
            'olajh': random.uniform(80, 110),        # Oil temp (°C)
            'olajnyomas': random.uniform(3, 8),      # Oil pressure (bar)
            'vizho': random.uniform(85, 95),         # Water temp (°C)
            'uzemanyagny': random.uniform(30, 100),  # Fuel level (%)
            'gyorsulas': random.uniform(-0.5, 1.5),  # Accel (g)
            'efogyaszt': random.uniform(5, 25),      # Consumption (L/h)
            'sebesseg': speed,                        # Speed (km/h)
            'aps': throttle,                          # Throttle (%)
            'feknyomas': random.uniform(50, 200),    # Brake pressure (bar)
            'etcmain': random.uniform(0, 100),       # ETC main (%)
            'fokozat': random.randint(0, 6),         # Gear (0-6)
            # Telemetry link data
            'rssi': random.uniform(-100, -50),       # RSSI (dBm)
            'snr': random.uniform(-5, 10),           # SNR (dB)
        }
        
        # Use app context to emit
        with app.app_context():
            socketio.emit('telemetry', telemetry, namespace='/')
        
        time.sleep(0.1)  # 100ms between packets = 10Hz

@app.route('/')
def index():
    return render_template('dashboard.html', 
                          google_maps_key=config['google_maps_api_key'])

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('response', {'data': 'Connected to telemetry server'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

if __name__ == '__main__':
    print('=' * 60)
    print('RaceLink Telemetry Server')
    print('=' * 60)
    print('Dashboard URL: http://localhost:5000/')
    print('=' * 60)
    print('Telemetry simulation with GPS enabled')
    print('Track center: {:.6f}, {:.6f}'.format(TRACK_CENTER_LAT, TRACK_CENTER_LON))
    print('=' * 60)
    
    # Start telemetry generator in background
    telemetry_thread = threading.Thread(target=generate_telemetry, daemon=True)
    telemetry_thread.start()
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)