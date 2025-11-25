from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial
import struct
import threading
import time
import json
import os
import math

# Load configuration
with open('config.json', 'r') as f:
    config = json.load(f)

# Create templates folder if it doesn't exist
os.makedirs('templates', exist_ok=True)

app = Flask(__name__, template_folder='templates')
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Serial port configuration
SERIAL_PORT = 'COM9'  # Change this to your RX's serial port
BAUD_RATE = 115200

# Packet framing
SYNC_BYTE_1 = 0xAA
SYNC_BYTE_2 = 0x55

# Telemetry packet structure - must match Arduino struct exactly
# Total size: 41 bytes
TELEM_STRUCT_FORMAT = '<BbBHHBBBBbBBBBBBBBiiBBBHbbB'
TELEM_STRUCT_SIZE = struct.calcsize(TELEM_STRUCT_FORMAT)

print(f"Expected packet size: {TELEM_STRUCT_SIZE} bytes")
print(f"Framed packet size: {TELEM_STRUCT_SIZE + 4} bytes (sync + size + data + checksum)")

# Field names matching the struct order
FIELD_NAMES = [
    'agt',           # uint8_t  - Air intake temp
    'lambda',        # int8_t   - Lambda
    'sebesseg',      # uint8_t  - Speed
    'feszultseg',    # uint16_t - Voltage
    'egt',           # uint16_t - Exhaust temp
    'fokozat',       # uint8_t  - Gear
    'olajny',        # uint8_t  - Oil pressure (olajnyomas)
    'olajh',         # uint8_t  - Oil temp
    'uzemanyagny',   # uint8_t  - Fuel level
    'gyorsulas',     # int8_t   - Acceleration
    'vizho',         # uint8_t  - Water temp
    'aps',           # uint8_t  - Throttle position
    'feknyomas',     # uint8_t  - Brake pressure
    'upshift',       # bool     - Upshift button
    'downshift',     # bool     - Downshift button
    'GPSSpeed',      # uint8_t  - GPS speed
    'GPSSats',       # uint8_t  - GPS satellites
    'GPSHDOP',       # uint8_t  - GPS HDOP
    'GPSLat',        # int32_t  - GPS latitude (scaled by 1000000)
    'GPSLng',        # int32_t  - GPS longitude (scaled by 1000000)
    'GPSHour',       # uint8_t  - GPS hour
    'GPSMinute',     # uint8_t  - GPS minute
    'GPSSecond',     # uint8_t  - GPS second
    'GPSMillisecond',# uint16_t - GPS millisecond
    'LoRaRssi',      # int8_t   - LoRa RSSI
    'LoRaSnr',       # int8_t   - LoRa SNR
    'LoRaPktRate'    # uint8_t  - LoRa packet rate
]

ser = None
connected_clients = 0
baseline_time = time.time()
last_lat = None
last_lon = None
last_heading = 0

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

def calculate_checksum(data):
    """Calculate XOR checksum"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def open_serial():
    """Open serial port connection"""
    global ser
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1  # Short timeout for responsive sync
        )
        print(f"✓ Serial port {SERIAL_PORT} opened at {BAUD_RATE} baud")
        return True
    except Exception as e:
        print(f"✗ Failed to open serial port: {e}")
        return False

def sync_to_packet(ser):
    """Find sync bytes in stream"""
    while True:
        byte = ser.read(1)
        if len(byte) == 0:
            return False
        if byte[0] == SYNC_BYTE_1:
            byte2 = ser.read(1)
            if len(byte2) > 0 and byte2[0] == SYNC_BYTE_2:
                return True

def parse_telemetry(raw_data):
    """Parse binary telemetry packet into dictionary"""
    global last_lat, last_lon, last_heading
    
    try:
        # Unpack binary data according to struct format
        values = struct.unpack(TELEM_STRUCT_FORMAT, raw_data)
        
        # Create dictionary with field names
        data = dict(zip(FIELD_NAMES, values))
        
        # Add timestamp as minutes:seconds.milliseconds
        current_time = time.time() - baseline_time
        minutes = int(current_time // 60)
        seconds = current_time % 60
        data['timestamp'] = f"{minutes}:{seconds:06.3f}"  # Format: M:SS.mmm
        data['timestamp_seconds'] = current_time  # Keep numeric for calculations
        
        # Convert GPS coordinates from scaled integers to floats
        if data['GPSLat'] != 0 and data['GPSLng'] != 0:
            lat = data['GPSLat'] / 1000000.0
            lon = data['GPSLng'] / 1000000.0
            data['latitude'] = lat
            data['longitude'] = lon
            
            # Calculate heading if we have previous position
            if last_lat is not None and last_lon is not None:
                if (lat != last_lat or lon != last_lon):
                    heading = calculate_bearing(last_lat, last_lon, lat, lon)
                    data['heading'] = heading
                    last_heading = heading
                else:
                    data['heading'] = last_heading
            else:
                data['heading'] = 0
            
            last_lat = lat
            last_lon = lon
        else:
            data['latitude'] = 0
            data['longitude'] = 0
            data['heading'] = 0
        
        # Convert voltage from millivolts to volts
        data['feszultseg'] = data['feszultseg'] / 1000.0
        
        # Convert lambda from int to float (assumed scaled by 10)
        data['lambda'] = data['lambda'] / 10.0
        
        # Convert HDOP from scaled integer to float
        data['GPSHDOP'] = data['GPSHDOP'] / 10.0
        
        # Convert acceleration from int to float (assumed scaled by 10)
        data['gyorsulas'] = data['gyorsulas'] / 10.0
        
        # Rename fields for dashboard compatibility
        data['rssi'] = data['LoRaRssi']
        data['snr'] = data['LoRaSnr']
        data['olajnyomas'] = data['olajny']  # Oil pressure
        
        return data
        
    except struct.error as e:
        print(f"✗ Struct unpacking error: {e}")
        return None

def uart_reader():
    """Background thread to read UART data with packet framing"""
    global ser
    
    if not open_serial():
        print("✗ Waiting 5 seconds before retry...")
        time.sleep(5)
        return
    
    print("✓ UART reader thread started")
    print("⏳ Waiting for packet sync...")
    
    packet_count = 0
    error_count = 0
    checksum_errors = 0
    sync_attempts = 0
    
    while True:
        try:
            if ser is None or not ser.is_open:
                print("✗ Serial port closed, attempting to reconnect...")
                time.sleep(2)
                if not open_serial():
                    continue
                sync_attempts = 0
            
            # Look for sync bytes
            if not sync_to_packet(ser):
                sync_attempts += 1
                if sync_attempts % 100 == 0:
                    print(f"⏳ Still searching for sync... ({sync_attempts} attempts)")
                continue
            
            if sync_attempts > 0:
                print(f"✓ Packet sync found after {sync_attempts} attempts!")
                sync_attempts = 0
            
            # Read size byte
            size_byte = ser.read(1)
            if len(size_byte) == 0:
                continue
            
            packet_size = size_byte[0]
            
            # Validate packet size
            if packet_size != TELEM_STRUCT_SIZE:
                print(f"⚠ Invalid packet size: {packet_size}, expected {TELEM_STRUCT_SIZE}")
                continue
            
            # Read data
            raw_data = ser.read(packet_size)
            if len(raw_data) != packet_size:
                print(f"⚠ Incomplete data: got {len(raw_data)}, expected {packet_size}")
                continue
            
            # Read checksum
            checksum_byte = ser.read(1)
            if len(checksum_byte) == 0:
                continue
            
            received_checksum = checksum_byte[0]
            calculated_checksum = calculate_checksum(raw_data)
            
            # Verify checksum
            if received_checksum != calculated_checksum:
                checksum_errors += 1
                if checksum_errors % 10 == 0:
                    print(f"✗ Checksum errors: {checksum_errors} (got 0x{received_checksum:02X}, expected 0x{calculated_checksum:02X})")
                continue
            
            # Parse the packet
            telemetry = parse_telemetry(raw_data)
            
            if telemetry:
                packet_count += 1
                
                # Emit to all connected clients
                with app.app_context():
                    socketio.emit('telemetry', telemetry, namespace='/')
                
                # Print status every 25 packets (1 second at 25Hz)
                if packet_count % 25 == 0:
                    print(f"📡 Pkts: {packet_count} | "
                          f"Speed: {telemetry['sebesseg']} km/h | "
                          f"Gear: {telemetry['fokozat']} | "
                          f"RSSI: {telemetry['rssi']} dBm | "
                          f"GPS: {telemetry['GPSSats']} sats | "
                          f"Pos: {telemetry['latitude']:.6f}, {telemetry['longitude']:.6f} | "
                          f"CRC errs: {checksum_errors}")
            else:
                error_count += 1
                if error_count % 10 == 0:
                    print(f"✗ Parse errors: {error_count}")
                
        except Exception as e:
            print(f"✗ Serial error: {e}")
            if ser:
                try:
                    ser.close()
                except:
                    pass
            ser = None
            time.sleep(2)
            sync_attempts = 0
            
        except KeyboardInterrupt:
            print("\n✓ Shutting down UART reader...")
            if ser:
                ser.close()
            break

@app.route('/')
def index():
    return render_template('dashboard.html', 
                          google_maps_key=config['google_maps_api_key'])

@socketio.on('connect')
def handle_connect():
    global connected_clients
    connected_clients += 1
    print(f'✓ Client connected (total: {connected_clients})')
    emit('response', {'data': 'Connected to RaceLink telemetry server'})

@socketio.on('disconnect')
def handle_disconnect():
    global connected_clients
    connected_clients -= 1
    print(f'✗ Client disconnected (total: {connected_clients})')

if __name__ == '__main__':
    print('=' * 70)
    print('🏎️  RaceLink UART Telemetry Server (Framed Protocol)')
    print('=' * 70)
    print(f'📡 Serial Port: {SERIAL_PORT} @ {BAUD_RATE} baud')
    print(f'📦 Data Size: {TELEM_STRUCT_SIZE} bytes')
    print(f'🔒 Frame Format: [0xAA][0x55][SIZE][DATA][CHECKSUM]')
    print(f'🌐 Dashboard URL: http://localhost:5000/')
    print('=' * 70)
    
    # Start UART reader in background thread
    uart_thread = threading.Thread(target=uart_reader, daemon=True)
    uart_thread.start()
    
    # Start Flask-SocketIO server
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n✓ Server shutting down...")
        if ser:
            ser.close()