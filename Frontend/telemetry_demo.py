from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import threading
import random
import time
import os

# Create templates folder if it doesn't exist
os.makedirs('templates', exist_ok=True)

app = Flask(__name__, template_folder='templates')
socketio = SocketIO(app, cors_allowed_origins="*")

# Simulate telemetry data
def generate_telemetry():
    """Generate realistic Formula Student telemetry data"""
    baseline_time = time.time()
    speed = 0
    throttle = 0
    
    while True:
        # Simulate acceleration/deceleration
        speed += random.uniform(-5, 8)
        speed = max(0, min(230, speed))  # Clamp 0-300 km/h
        
        throttle += random.uniform(-10, 15)
        throttle = max(0, min(100, throttle))  # Clamp 0-100%
        
        telemetry = {
            'timestamp': int((time.time() - baseline_time) * 1000),  # ms
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
            'rssi': random.uniform(-100, -50),       # RSSI (dBm)
            'snr': random.uniform(-5, 10),           # SNR (dB)
        }
        
        # Use app context to emit
        with app.app_context():
            socketio.emit('telemetry', telemetry, namespace='/')
        
        time.sleep(0.1)  # 100ms between packets

@app.route('/')
def index():
    return render_template('dashboard.html')
    
@app.route('/golden')
def golden():
    return render_template('dashboard_golden.html')

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('response', {'data': 'Connected to telemetry server'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

if __name__ == '__main__':
    # Start telemetry generator in background
    telemetry_thread = threading.Thread(target=generate_telemetry, daemon=True)
    telemetry_thread.start()
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)

