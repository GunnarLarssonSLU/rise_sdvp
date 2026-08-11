# RISE SDVP Web Interface

A Streamlit-based web application that replicates the behavior of the farmTable, fieldTable, and pathTable from the RControlStation Qt application.

## Overview

This web application provides a modern, browser-based interface for managing farms, fields, and paths in the RISE SDVP system. It communicates with the same web server endpoints used by the original Qt application.

## Features

- **Farm Management**: View, add, and delete farms with their coordinates and connection details
- **Field Management**: Manage fields associated with farms, including fencing status and file associations
- **Path Management**: Handle paths within fields for navigation and planning
- **Hierarchical Navigation**: Tab-based interface that follows the farm → field → path hierarchy
- **Real-time Data**: Fetches data from the web server and displays it in interactive tables
- **Responsive Design**: Works on desktop and mobile devices

## Installation

```bash
# Clone the repository or navigate to the web directory
cd /home/gunnar/code/rise_sdvp/web

# Install dependencies
pip install -r requirements.txt
```

## Usage

```bash
# Run the Streamlit application
streamlit run streamlit_app.py
```

The application will start and open in your default web browser at `http://localhost:8501`.

## Requirements

- Python 3.8 or higher
- Streamlit 1.29.0 or higher
- requests library
- pandas library
- A running instance of the RISE SDVP web server at `http://127.0.0.1:8080`

## Configuration

The application connects to the web server at `http://127.0.0.1:8080` by default. If your server is running on a different address, modify the `SERVER_URL` constant in `streamlit_app.py`.

## API Endpoints Used

The application uses the following endpoints from the RISE SDVP web server:

- `GET /all_farms` - Retrieve all farms/locations
- `GET /all_fields?farm_id={id}` - Retrieve fields for a specific farm
- `GET /all_paths?field_id={id}` - Retrieve paths for a specific field
- `GET /add_farm?name={name}&longitude={lon}&latitude={lat}` - Add a new farm
- `GET /add_field?name={name}&farm_id={id}&fenced={fenced}` - Add a new field
- `GET /add_path?name={name}&field_id={id}` - Add a new path
- `GET /remove_farm?id={id}` - Delete a farm
- `GET /remove_field?id={id}` - Delete a field
- `GET /remove_path?id={id}` - Delete a path

## Application Structure

- `streamlit_app.py` - Main application file containing the FarmManagerApp class
- `requirements.txt` - Python dependencies
- `README.md` - This file

## Behavior Comparison with Qt Application

This Streamlit application replicates the following behavior from the original Qt RControlStation:

### Farm Table (farmTable)
- Displays farm name, longitude, latitude, IP, port, NTRIP, user, password, stream, autoconnect
- Hides IP, port, NTRIP, user, password, stream, autoconnect columns (as in the Qt version)
- Supports selection of farms
- Supports adding and deleting farms
- Updates map reference when a farm is selected

### Field Table (fieldTable)
- Displays field name, fenced status, and file information
- Hides location and ID columns (as in the Qt version)
- Uses checkboxes for the fenced column
- Supports selection of fields
- Supports adding and deleting fields
- Updates field display on the map when selected

### Path Table (pathTable)
- Displays path name
- Hides ID and field columns (as in the Qt version)
- Supports selection of paths
- Supports adding and deleting paths
- Updates path display on the map when selected

## Keyboard Shortcuts

The original Qt application supports keyboard shortcuts like:
- Delete key to delete selected items
- F5 or Ctrl+R to refresh tables
- Enter key to save changes

In the Streamlit version, these are replaced with buttons for better web compatibility.

## Limitations

- Streamlit's stateless nature means the application relies on session state for persistence
- Real-time updates require manual refresh (unlike the Qt version which may have automatic updates)
- Some advanced features from the Qt version (like direct XML editing) are not implemented
- Keyboard shortcuts are limited compared to the desktop application

## Future Enhancements

- Add support for editing existing farms, fields, and paths
- Implement automatic refresh with polling
- Add more detailed information display
- Support for importing/exporting data
- Integration with mapping components for visual display
- Add user authentication and authorization

## License

This application is part of the RISE SDVP project and inherits its licensing terms.