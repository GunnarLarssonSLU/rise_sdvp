import subprocess
import sqlite3
import xml.etree.ElementTree as ET
from flask import Flask, Response, request

app = Flask(__name__)


@app.route('/')
def hello_world():
    return "Hello, World!"


@app.route('/machines')
def list_machines():
    try:
        result = subprocess.run(
            ['nmap', '-sP', '192.168.200.3/24'],
            capture_output=True,
            text=True,
            timeout=30
        )
        # Parse output to extract IP addresses
        import re
        ip_pattern = r'\b(?:\d{1,3}\.){3}\d{1,3}\b'
        ips = re.findall(ip_pattern, result.stdout)
        # Remove duplicates and 192.168.200.3, then sort
        #unique_ips = sorted(set(ips) - {'192.168.200.3'})
        unique_ips = sorted(set(ips))
        
        # Look up names and vehicle types from database
        conn = sqlite3.connect('data.db')
        cursor = conn.cursor()
        
        # Create XML structure
        root = ET.Element('machines')
        for ip in unique_ips:
            cursor.execute('SELECT name, iVehicletype FROM machines WHERE ip = ?', (ip,))
            row = cursor.fetchone()
            conn.commit()
            if row:
                machine_elem = ET.SubElement(root, 'machine')
                ET.SubElement(machine_elem, 'name').text = row[0]
                ET.SubElement(machine_elem, 'ip').text = ip
                if row[1] is not None:
                    ET.SubElement(machine_elem, 'iVehicletype').text = str(row[1])
        
        conn.close()
        
        # Convert to XML string with declaration
        xml_str = '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode')
        
        # Write to file
        with open('found_machines.xml', 'w') as f:
            f.write(xml_str)
        
        return Response(xml_str, mimetype='application/xml')
    except subprocess.TimeoutExpired:
        return "Command timed out", 500
    except Exception as e:
        return f"Error: {str(e)}", 500


@app.route('/all_machines')
def all_machines():
    try:
        # Connect to SQLite database
        conn = sqlite3.connect('data.db')
        cursor = conn.cursor()
        
        # Query all machines from the table
        cursor.execute('SELECT id, name, ip, iVehicletype FROM machines')
        machines = cursor.fetchall()
        conn.close()
        
        # Create XML structure
        root = ET.Element('machines')
        for machine_id, name, ip, iVehicletype in machines:
            machine_elem = ET.SubElement(root, 'machine')
            ET.SubElement(machine_elem, 'id').text = str(machine_id)
            ET.SubElement(machine_elem, 'name').text = name
            ET.SubElement(machine_elem, 'ip').text = ip
            if iVehicletype is not None:
                ET.SubElement(machine_elem, 'iVehicletype').text = str(iVehicletype)
        
        # Convert to XML string with declaration
        xml_str = '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode')
        
        # Write to file
        with open('machines.xml', 'w') as f:
            f.write(xml_str)
        
        return Response(xml_str, mimetype='application/xml')
    except Exception as e:
        return f"Error: {str(e)}", 500


@app.route('/add_machine', methods=['POST'])
def add_machine():
    try:
        # Debug: Print request information
        print(f"DEBUG: Request method: {request.method}")
        print(f"DEBUG: Request form data: {dict(request.form)}")
        print(f"DEBUG: Request args (URL params): {dict(request.args)}")
        print(f"DEBUG: Request data: {request.data}")
        
        # Get all the fields from POST form data or URL parameters
        def get_param(name):
            value = request.form.get(name) or request.args.get(name)
            print(f"DEBUG: get_param('{name}') = {value}")
            return value
        
        name = get_param('name')
        ip = get_param('ip')
        port = get_param('port')
        gearratio = get_param('gearratio')
        wheeldiameter_m = get_param('wheeldiameter_m')
        motorpoles = get_param('motorpoles')
        turnradius_m = get_param('turnradius_m')
        steeringramp = get_param('steeringramp')
        axisdistance_m = get_param('axisdistance_m')
        servocenter = get_param('servocenter')
        servorange = get_param('servorange')
        yawIMUgain = get_param('yawIMUgain')
        servoPgain = get_param('servoPgain')
        servoIgain = get_param('servoIgain')
        servoDgain = get_param('servoDgain')
        maxleft_degrees = get_param('maxleft_degrees')
        maxright_degrees = get_param('maxright_degrees')
        centervoltage_V = get_param('centervoltage_V')
        iVehicletype = get_param('iVehicletype') or get_param('vehicle_type_id')
        
        print(f"DEBUG: Final values - name: {name}, ip: {ip}, iVehicletype: {iVehicletype}")
        
        # Validate required fields
        if not name:
            print("DEBUG: Missing name field")
            return "Error: 'name' field is required", 400

        if not ip:
            print("DEBUG: Missing ip field")
            return "Error: 'ip' field is required", 400

        # Connect to database
        print("DEBUG: Connecting to database")
        conn = sqlite3.connect('data.db')
        cursor = conn.cursor()
        
        # Build the INSERT query with all fields
        columns = ['name']
        values = [name]
        placeholders = ['?']
        
        # Add optional fields if provided
        field_mappings = [
            ('ip', ip),
            ('port', port),
            ('gearratio', gearratio),
            ('wheeldiameter_m', wheeldiameter_m),
            ('motorpoles', motorpoles),
            ('turnradius_m', turnradius_m),
            ('steeringramp', steeringramp),
            ('axisdistance_m', axisdistance_m),
            ('servocenter', servocenter),
            ('servorange', servorange),
            ('yawIMUgain', yawIMUgain),
            ('servoPgain', servoPgain),
            ('servoIgain', servoIgain),
            ('servoDgain', servoDgain),
            ('maxleft_degrees', maxleft_degrees),
            ('maxright_degrees', maxright_degrees),
            ('centervoltage_V', centervoltage_V),
            ('iVehicletype', iVehicletype)
        ]
        
        for field_name, field_value in field_mappings:
            if field_value is not None:
                columns.append(field_name)
                values.append(field_value)
                placeholders.append('?')
        
        # Create and execute the INSERT statement
        columns_str = ', '.join(columns)
        placeholders_str = ', '.join(placeholders)
        query = f'INSERT INTO machines ({columns_str}) VALUES ({placeholders_str})'
        print(f"DEBUG: Executing query: {query}")
        print(f"DEBUG: With values: {values}")
        cursor.execute(query, values)
        print(f"DEBUG: Rows affected: {cursor.rowcount}")
        
        # Get the ID of the newly inserted machine
        machine_id = cursor.lastrowid
        
        # Log the addition to log_machines table
        from datetime import datetime
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Build the machine data as a string for logging
        machine_data = {col: val for col, val in zip(columns, values)}
        machine_data_str = str(machine_data)
        
        cursor.execute(
            'INSERT INTO log_machines (machine_id, machine_data, timestamp, action) VALUES (?, ?, ?, ?)',
            (machine_id, machine_data_str, timestamp, 'ADD')
        )
        
        conn.commit()
        conn.close()
        
        print("DEBUG: Machine added successfully")
        return "Machine added successfully", 201
        
    except sqlite3.IntegrityError as e:
        print(f"DEBUG: IntegrityError: {str(e)}")
        return f"Error: {str(e)}", 409
    except Exception as e:
        print(f"DEBUG: Exception: {str(e)}")
        import traceback
        print(f"DEBUG: Traceback: {traceback.format_exc()}")
        return f"Error: {str(e)}", 500


@app.route('/remove_machine', methods=['GET', 'POST'])
def remove_machine():
    try:
        # Get all the fields from POST form data or URL parameters (same as add_machine)
        def get_param(name):
            value = request.form.get(name) or request.args.get(name)
            return value
        
        machine_id = get_param('id')
        
        if not machine_id:
            return "Error: 'id' field is required", 400
        
        # Connect to database
        conn = sqlite3.connect('data.db')
        cursor = conn.cursor()
        
        # First, get the machine data before deleting
        cursor.execute('SELECT * FROM machines WHERE id = ?', (machine_id,))
        machine = cursor.fetchone()
        
        if machine is None:
            conn.close()
            return f"Error: No machine found with id {machine_id}", 404
        
        # Build machine data string for logging
        from datetime import datetime
        columns = [description[0] for description in cursor.description]
        machine_data = dict(zip(columns, machine))
        machine_data_str = str(machine_data)
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Log the removal to log_machines table
        cursor.execute(
            'INSERT INTO log_machines (machine_id, machine_data, timestamp, action) VALUES (?, ?, ?, ?)',
            (machine_id, machine_data_str, timestamp, 'REMOVE')
        )
        
        # Delete the machine with the specified id
        cursor.execute('DELETE FROM machines WHERE id = ?', (machine_id,))
        
        conn.commit()
        conn.close()
        
        return f"Machine with id {machine_id} removed successfully", 200
        
    except Exception as e:
        return f"Error: {str(e)}", 500


@app.route('/vehicle_types')
def vehicle_types():
    try:
        # Connect to SQLite database
        conn = sqlite3.connect('data.db')
        cursor = conn.cursor()
        
        # Query all vehicle types from the table
        cursor.execute('SELECT id, name, typeofsteering, length_m, width_m FROM vehicle_types')
        vehicle_types = cursor.fetchall()
        conn.close()
        
        # Create XML structure
        root = ET.Element('vehicle_types')
        for vehicle_type in vehicle_types:
            vt_elem = ET.SubElement(root, 'vehicle_type')
            ET.SubElement(vt_elem, 'id').text = str(vehicle_type[0])
            ET.SubElement(vt_elem, 'name').text = str(vehicle_type[1])
            ET.SubElement(vt_elem, 'typeofsteering').text = str(vehicle_type[2])
            ET.SubElement(vt_elem, 'length_m').text = str(vehicle_type[3])
            ET.SubElement(vt_elem, 'width_m').text = str(vehicle_type[4])
        
        # Convert to XML string with declaration
        xml_str = '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode')
        
        # Write to file
        with open('vehicle_types.xml', 'w') as f:
            f.write(xml_str)
        
        return Response(xml_str, mimetype='application/xml')
    except Exception as e:
        return f"Error: {str(e)}", 500


@app.route('/all_farms')
def all_farms():
    try:
        # Connect to SQLite database
        conn = sqlite3.connect('data.db')
        cursor = conn.cursor()
        
        # Query all fields from the locations table
        cursor.execute('SELECT * FROM locations')
        locations = cursor.fetchall()
        
        # Get column names from cursor description
        column_names = [description[0] for description in cursor.description]
        conn.close()
        
        # Create XML structure
        root = ET.Element('locations')
        for location in locations:
            location_elem = ET.SubElement(root, 'location')
            for i, column_name in enumerate(column_names):
                ET.SubElement(location_elem, column_name).text = str(location[i])
        
        # Convert to XML string with declaration
        xml_str = '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode')
        
        # Write to file
        with open('locations.xml', 'w') as f:
            f.write(xml_str)
        
        return Response(xml_str, mimetype='application/xml')
    except Exception as e:
        return f"Error: {str(e)}", 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
