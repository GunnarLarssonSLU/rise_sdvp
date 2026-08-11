#!/usr/bin/env python3
"""
Streamlit application that replicates the behavior of farmTable, fieldTable, and pathTable
from the RControlStation application.

This app provides a web-based interface for managing farms, fields, and paths,
communicating with the same web server endpoints used by the Qt application.
"""

import streamlit as st
import requests
import xml.etree.ElementTree as ET
from typing import List, Dict, Any, Optional
import pandas as pd
from datetime import datetime

# Configuration
SERVER_URL = "http://127.0.0.1:8080"
TIMEOUT = 10  # seconds

# Set page configuration
st.set_page_config(
    page_title="RISE SDVP - Farm Management",
    page_icon="🚜",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for better styling
st.markdown("""
    <style>
    .main {
        padding: 0rem 1rem;
    }
    .stDataFrame {
        border: 1px solid #ddd;
        border-radius: 5px;
    }
    .stButton button {
        border-radius: 5px;
        padding: 0.25rem 1rem;
    }
    .highlight-row {
        background-color: #f0f2f6 !important;
    }
    </style>
""", unsafe_allow_html=True)


class FarmManagerApp:
    """Main application class for managing farms, fields, and paths."""
    
    def __init__(self):
        self.farms: List[Dict[str, Any]] = []
        self.fields: List[Dict[str, Any]] = []
        self.paths: List[Dict[str, Any]] = []
        
        # Initialize session state
        self._initialize_session_state()
        
    def _initialize_session_state(self):
        """Initialize Streamlit session state variables."""
        if 'farms' not in st.session_state:
            st.session_state.farms = []
        if 'fields' not in st.session_state:
            st.session_state.fields = []
        if 'paths' not in st.session_state:
            st.session_state.paths = []
        if 'selected_farm_id' not in st.session_state:
            st.session_state.selected_farm_id = None
        if 'selected_field_id' not in st.session_state:
            st.session_state.selected_field_id = None
        if 'selected_path_id' not in st.session_state:
            st.session_state.selected_path_id = None
        if 'last_refresh_time' not in st.session_state:
            st.session_state.last_refresh_time = None
        if 'error_message' not in st.session_state:
            st.session_state.error_message = None
        if 'success_message' not in st.session_state:
            st.session_state.success_message = None
            
    def fetch_all_farms(self) -> List[Dict[str, Any]]:
        """Fetch all farms from the web server."""
        try:
            url = f"{SERVER_URL}/all_farms"
            response = requests.get(url, timeout=TIMEOUT)
            
            if response.status_code == 200:
                xml_data = response.content
                farms = self._parse_farms_xml(xml_data)
                st.session_state.farms = farms
                st.session_state.last_refresh_time = datetime.now()
                return farms
            else:
                st.session_state.error_message = f"HTTP Error {response.status_code}: {response.text}"
                return []
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return []
            
    def fetch_fields_for_farm(self, farm_id: int) -> List[Dict[str, Any]]:
        """Fetch all fields for a specific farm."""
        try:
            url = f"{SERVER_URL}/all_fields?farm={farm_id}"
            response = requests.get(url, timeout=TIMEOUT)
            
            if response.status_code == 200:
                xml_data = response.content
                fields = self._parse_fields_xml(xml_data)
                st.session_state.fields = fields
                return fields
            else:
                st.session_state.error_message = f"HTTP Error {response.status_code}: {response.text}"
                return []
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return []
            
    def fetch_paths_for_field(self, field_id: int) -> List[Dict[str, Any]]:
        """Fetch all paths for a specific field."""
        try:
            url = f"{SERVER_URL}/all_paths?field_id={field_id}"
            response = requests.get(url, timeout=TIMEOUT)
            
            if response.status_code == 200:
                xml_data = response.content
                paths = self._parse_paths_xml(xml_data)
                st.session_state.paths = paths
                return paths
            else:
                st.session_state.error_message = f"HTTP Error {response.status_code}: {response.text}"
                return []
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return []
            
    def _parse_farms_xml(self, xml_data: bytes) -> List[Dict[str, Any]]:
        """Parse XML data containing farm/location information."""
        farms = []
        try:
            root = ET.fromstring(xml_data)
            
            for location_elem in root.findall('.//location'):
                farm = {
                    'id': None,
                    'name': '',
                    'longitude': 0.0,
                    'latitude': 0.0,
                    'ip': '',
                    'port': '',
                    'ntrip': '',
                    'user': '',
                    'password': '',
                    'stream': '',
                    'autoconnect': False
                }
                
                for child in location_elem:
                    tag = child.tag
                    text = child.text or ''
                    
                    if tag == 'id':
                        farm['id'] = int(text) if text.isdigit() else None
                    elif tag == 'name':
                        farm['name'] = text
                    elif tag == 'longitude':
                        farm['longitude'] = float(text) if text else 0.0
                    elif tag == 'latitude':
                        farm['latitude'] = float(text) if text else 0.0
                    elif tag == 'ip':
                        farm['ip'] = text
                    elif tag == 'port':
                        farm['port'] = text
                    elif tag == 'NTRIP':
                        farm['ntrip'] = text
                    elif tag == 'user':
                        farm['user'] = text
                    elif tag == 'password':
                        farm['password'] = text
                    elif tag == 'stream':
                        farm['stream'] = text
                    elif tag == 'autoconnect':
                        farm['autoconnect'] = text.lower() in ['1', 'true', 'yes']
                
                if farm['name']:  # Only add if we have a name
                    farms.append(farm)
                    
        except ET.ParseError as e:
            st.session_state.error_message = f"XML parsing error: {str(e)}"
            
        return farms
        
    def _parse_fields_xml(self, xml_data: bytes) -> List[Dict[str, Any]]:
        """Parse XML data containing field information."""
        fields = []
        try:
            root = ET.fromstring(xml_data)
            
            for field_elem in root.findall('.//field'):
                field = {
                    'id': None,
                    'name': '',
                    'fenced': False,
                    'storedinfile': '',
                    'location': None
                }
                
                for child in field_elem:
                    tag = child.tag
                    text = child.text or ''
                    
                    if tag == 'id':
                        field['id'] = int(text) if text.isdigit() else None
                    elif tag == 'name':
                        field['name'] = text
                    elif tag == 'fenced':
                        field['fenced'] = text.lower() in ['1', 'true', 'yes']
                    elif tag == 'storedinfile':
                        field['storedinfile'] = text
                    elif tag == 'location':
                        field['location'] = int(text) if text.isdigit() else None
                
                if field['name']:  # Only add if we have a name
                    fields.append(field)
                    
        except ET.ParseError as e:
            st.session_state.error_message = f"XML parsing error: {str(e)}"
            
        return fields
        
    def _parse_paths_xml(self, xml_data: bytes) -> List[Dict[str, Any]]:
        """Parse XML data containing path information."""
        paths = []
        try:
            root = ET.fromstring(xml_data)
            
            for path_elem in root.findall('.//path'):
                path = {
                    'id': None,
                    'name': '',
                    'field': None
                }
                
                for child in path_elem:
                    tag = child.tag
                    text = child.text or ''
                    
                    if tag == 'id':
                        path['id'] = int(text) if text.isdigit() else None
                    elif tag == 'name':
                        path['name'] = text
                    elif tag == 'field':
                        path['field'] = int(text) if text.isdigit() else None
                
                if path['name']:  # Only add if we have a name
                    paths.append(path)
                    
        except ET.ParseError as e:
            st.session_state.error_message = f"XML parsing error: {str(e)}"
            
        return paths
        
    def add_farm(self, name: str, longitude: float = 0.0, latitude: float = 0.0) -> bool:
        """Add a new farm to the server."""
        try:
            url = f"{SERVER_URL}/add_farm"
            params = {
                'name': name,
                'longitude': longitude,
                'latitude': latitude
            }
            response = requests.get(url, params=params, timeout=TIMEOUT)
            
            if response.status_code == 200:
                st.session_state.success_message = f"Farm '{name}' added successfully"
                return True
            else:
                st.session_state.error_message = f"Failed to add farm: HTTP {response.status_code}"
                return False
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return False
            
    def add_field(self, name: str, farm_id: int, fenced: bool = False) -> bool:
        """Add a new field to the server."""
        try:
            url = f"{SERVER_URL}/add_field"
            params = {
                'name': name,
                'farm_id': farm_id,
                'fenced': '1' if fenced else '0'
            }
            response = requests.get(url, params=params, timeout=TIMEOUT)
            
            if response.status_code == 200:
                st.session_state.success_message = f"Field '{name}' added successfully"
                return True
            else:
                st.session_state.error_message = f"Failed to add field: HTTP {response.status_code}"
                return False
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return False
            
    def add_path(self, name: str, field_id: int) -> bool:
        """Add a new path to the server."""
        try:
            url = f"{SERVER_URL}/add_path"
            params = {
                'name': name,
                'field_id': field_id
            }
            response = requests.get(url, params=params, timeout=TIMEOUT)
            
            if response.status_code == 200:
                st.session_state.success_message = f"Path '{name}' added successfully"
                return True
            else:
                st.session_state.error_message = f"Failed to add path: HTTP {response.status_code}"
                return False
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return False
            
    def delete_farm(self, farm_id: int) -> bool:
        """Delete a farm from the server."""
        try:
            url = f"{SERVER_URL}/remove_farm"
            params = {'id': farm_id}
            response = requests.get(url, params=params, timeout=TIMEOUT)
            
            if response.status_code == 200:
                st.session_state.success_message = f"Farm with ID {farm_id} deleted successfully"
                return True
            else:
                st.session_state.error_message = f"Failed to delete farm: HTTP {response.status_code}"
                return False
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return False
            
    def delete_field(self, field_id: int) -> bool:
        """Delete a field from the server."""
        try:
            url = f"{SERVER_URL}/remove_field"
            params = {'id': field_id}
            response = requests.get(url, params=params, timeout=TIMEOUT)
            
            if response.status_code == 200:
                st.session_state.success_message = f"Field with ID {field_id} deleted successfully"
                return True
            else:
                st.session_state.error_message = f"Failed to delete field: HTTP {response.status_code}"
                return False
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return False
            
    def delete_path(self, path_id: int) -> bool:
        """Delete a path from the server."""
        try:
            url = f"{SERVER_URL}/remove_path"
            params = {'id': path_id}
            response = requests.get(url, params=params, timeout=TIMEOUT)
            
            if response.status_code == 200:
                st.session_state.success_message = f"Path with ID {path_id} deleted successfully"
                return True
            else:
                st.session_state.error_message = f"Failed to delete path: HTTP {response.status_code}"
                return False
                
        except requests.exceptions.RequestException as e:
            st.session_state.error_message = f"Network error: {str(e)}"
            return False
    def render_farm_table(self):
        """Render the farm table with selection and actions."""
        st.header("🏠 Farms")
        
        # Display error or success messages
        if st.session_state.error_message:
            st.error(st.session_state.error_message)
            st.session_state.error_message = None
            
        if st.session_state.success_message:
            st.success(st.session_state.success_message)
            st.session_state.success_message = None
        
        # Refresh button and Add Farm button
        col1, col2, col3 = st.columns([1, 2, 1])
        with col1:
            if st.button("🔄 Refresh", width='stretch'):
                self.fetch_all_farms()
                st.rerun()
        
        with col3:
            if st.button("➕ Add Farm", width='stretch'):
                st.session_state.show_add_farm = True
        
        # Display farm table
        if st.session_state.farms:
            farm_data = []
            for farm in st.session_state.farms:
                farm_data.append({
                    'ID': farm['id'],
                    'Name': farm['name'],
                    'Longitude': f"{farm['longitude']:.14f}",
                    'Latitude': f"{farm['latitude']:.14f}",
                    'IP': farm['ip'],
                    'Port': farm['port']
                })
            
            df_farms = pd.DataFrame(farm_data)
            
            # Display table
            st.dataframe(
                df_farms,
                width='stretch',
                height=300,
                hide_index=True,
                column_config={
                    "ID": st.column_config.NumberColumn(width="small"),
                    "Name": st.column_config.TextColumn(width="medium"),
                    "Longitude": st.column_config.NumberColumn(format="%.14f"),
                    "Latitude": st.column_config.NumberColumn(format="%.14f"),
                    "IP": st.column_config.TextColumn(width="small"),
                    "Port": st.column_config.TextColumn(width="small")
                },
                use_container_width=True
            )
            
            # Handle row selection via click - use a simple approach with buttons in the table
            # Create a selection interface by showing farm names as buttons
            for farm in st.session_state.farms:
                if st.button(f"Select: {farm['name']}", width='stretch', key=f"select_farm_{farm['id']}"):
                    st.session_state.selected_farm_id = farm['id']
                    st.rerun()
            
            # Show delete button for selected farm
            if st.session_state.selected_farm_id:
                selected_farm = next((f for f in st.session_state.farms if f['id'] == st.session_state.selected_farm_id), None)
                if selected_farm:
                    if st.button(f"🗑️ Delete '{selected_farm['name']}'", key=f"delete_farm_{selected_farm['id']}"):
                        if self.delete_farm(selected_farm['id']):
                            self.fetch_all_farms()
                            st.session_state.selected_farm_id = None
                            st.rerun()
        else:
            st.info("No farms found. Click 'Refresh' to load data.")
            
        # Add farm dialog
        if 'show_add_farm' in st.session_state and st.session_state.show_add_farm:
            with st.expander("Add New Farm", expanded=True):
                farm_name = st.text_input("Farm Name:", key="new_farm_name")
                col1, col2 = st.columns(2)
                with col1:
                    farm_longitude = st.number_input("Longitude:", value=0.0, format="%.14f", key="new_farm_longitude")
                with col2:
                    farm_latitude = st.number_input("Latitude:", value=0.0, format="%.14f", key="new_farm_latitude")
                
                col1, col2 = st.columns(2)
                with col1:
                    if st.button("Add Farm", width='stretch'):
                        if farm_name:
                            if self.add_farm(farm_name, farm_longitude, farm_latitude):
                                st.session_state.show_add_farm = False
                                self.fetch_all_farms()
                                st.rerun()
                        else:
                            st.error("Please enter a farm name")
                with col2:
                    if st.button("Cancel", width='stretch'):
                        st.session_state.show_add_farm = False
                        st.rerun()
                            
        return st.session_state.selected_farm_id
        
    def render_field_table(self, farm_id: Optional[int] = None):
        """Render the field table for the selected farm."""
        if farm_id is None:
            st.info("Please select a farm to view its fields.")
            return None
        
        st.header("🌾 Fields")
        
        # Fetch fields for the selected farm
        if farm_id != st.session_state.get('last_farm_id'):
            self.fetch_fields_for_farm(farm_id)
            st.session_state.last_farm_id = farm_id
        
        # Refresh and add buttons
        col1, col2, col3 = st.columns([1, 2, 1])
        with col1:
            if st.button("🔄 Refresh", width='stretch', key=f"refresh_fields_{farm_id}"):
                self.fetch_fields_for_farm(farm_id)
                st.rerun()
        
        with col3:
            if st.button("➕ Add Field", width='stretch', key=f"add_field_{farm_id}"):
                st.session_state.show_add_field = True
        
        # Display field table
        if st.session_state.fields:
            # Filter fields by farm
            farm_fields = [f for f in st.session_state.fields if f.get('location') == farm_id]
            
            if farm_fields:
                field_data = []
                for field in farm_fields:
                    field_data.append({
                        'ID': field['id'],
                        'Name': field['name'],
                        'Fenced': '✅' if field['fenced'] else '❌',
                        'File': field['storedinfile'][:50] + '...' if len(field['storedinfile']) > 50 else field['storedinfile']
                    })
                
                df_fields = pd.DataFrame(field_data)
                
                # Display table
                st.dataframe(
                    df_fields,
                    width='stretch',
                    height=250,
                    hide_index=True,
                    column_config={
                        "ID": st.column_config.NumberColumn(width="small"),
                        "Name": st.column_config.TextColumn(width="medium"),
                        "Fenced": st.column_config.TextColumn(width="small"),
                        "File": st.column_config.TextColumn(width="large")
                    },
                    use_container_width=True
                )
                
                # Handle row selection via buttons
                for field in farm_fields:
                    if st.button(f"Select: {field['name']}", width='stretch', key=f"select_field_{field['id']}"):
                        st.session_state.selected_field_id = field['id']
                        st.rerun()
                
                # Show delete button for selected field
                if st.session_state.selected_field_id:
                    selected_field = next((f for f in farm_fields if f['id'] == st.session_state.selected_field_id), None)
                    if selected_field:
                        if st.button(f"🗑️ Delete '{selected_field['name']}'", key=f"delete_field_{selected_field['id']}"):
                            if self.delete_field(selected_field['id']):
                                self.fetch_fields_for_farm(farm_id)
                                st.session_state.selected_field_id = None
                                st.rerun()
            else:
                st.info(f"No fields found for farm ID {farm_id}.")
        else:
            st.info("No fields found. Click 'Refresh' to load data.")
        
        # Add field dialog
        if 'show_add_field' in st.session_state and st.session_state.show_add_field:
            with st.expander("Add New Field", expanded=True):
                field_name = st.text_input("Field Name:", key="new_field_name")
                field_fenced = st.checkbox("Is Fenced?", value=False, key="new_field_fenced")
                
                col1, col2 = st.columns(2)
                with col1:
                    if st.button("Add Field", width='stretch'):
                        if field_name:
                            if self.add_field(field_name, farm_id, field_fenced):
                                st.session_state.show_add_field = False
                                self.fetch_fields_for_farm(farm_id)
                                st.rerun()
                        else:
                            st.error("Please enter a field name")
                with col2:
                    if st.button("Cancel", width='stretch'):
                        st.session_state.show_add_field = False
                        st.rerun()
                            
        return st.session_state.selected_field_id
        
    def render_path_table(self, field_id: Optional[int] = None):
        """Render the path table for the selected field."""
        if field_id is None:
            st.info("Please select a field to view its paths.")
            return None
        
        st.header("🛣️ Paths")
        
        # Fetch paths for the selected field
        if field_id != st.session_state.get('last_field_id'):
            self.fetch_paths_for_field(field_id)
            st.session_state.last_field_id = field_id
        
        # Refresh and add buttons
        col1, col2, col3 = st.columns([1, 2, 1])
        with col1:
            if st.button("🔄 Refresh", width='stretch', key=f"refresh_paths_{field_id}"):
                self.fetch_paths_for_field(field_id)
                st.rerun()
        
        with col3:
            if st.button("➕ Add Path", width='stretch', key=f"add_path_{field_id}"):
                st.session_state.show_add_path = True
        
        # Display path table
        if st.session_state.paths:
            # Filter paths by field
            field_paths = [p for p in st.session_state.paths if p.get('field') == field_id]
            
            if field_paths:
                path_data = []
                for path in field_paths:
                    path_data.append({
                        'ID': path['id'],
                        'Name': path['name'],
                        'Field ID': path['field']
                    })
                
                df_paths = pd.DataFrame(path_data)
                
                # Display table
                st.dataframe(
                    df_paths,
                    width='stretch',
                    height=200,
                    hide_index=True,
                    column_config={
                        "ID": st.column_config.NumberColumn(width="small"),
                        "Name": st.column_config.TextColumn(width="medium"),
                        "Field ID": st.column_config.NumberColumn(width="small")
                    },
                    use_container_width=True
                )
                
                # Handle row selection via buttons
                for path in field_paths:
                    if st.button(f"Select: {path['name']}", width='stretch', key=f"select_path_{path['id']}"):
                        st.session_state.selected_path_id = path['id']
                        st.rerun()
                
                # Show delete button for selected path
                if st.session_state.selected_path_id:
                    selected_path = next((p for p in field_paths if p['id'] == st.session_state.selected_path_id), None)
                    if selected_path:
                        if st.button(f"🗑️ Delete '{selected_path['name']}'", key=f"delete_path_{selected_path['id']}"):
                            if self.delete_path(selected_path['id']):
                                self.fetch_paths_for_field(field_id)
                                st.session_state.selected_path_id = None
                                st.rerun()
            else:
                st.info(f"No paths found for field ID {field_id}.")
        else:
            st.info("No paths found. Click 'Refresh' to load data.")
        
        # Add path dialog
        if 'show_add_path' in st.session_state and st.session_state.show_add_path:
            with st.expander("Add New Path", expanded=True):
                path_name = st.text_input("Path Name:", key="new_path_name")
                
                col1, col2 = st.columns(2)
                with col1:
                    if st.button("Add Path", width='stretch'):
                        if path_name:
                            if self.add_path(path_name, field_id):
                                st.session_state.show_add_path = False
                                self.fetch_paths_for_field(field_id)
                                st.rerun()
                        else:
                            st.error("Please enter a path name")
                with col2:
                    if st.button("Cancel", width='stretch'):
                        st.session_state.show_add_path = False
                        st.rerun()
                            
        return st.session_state.selected_path_id
        
    def render_summary(self):
        """Render a summary of the current selection."""
        st.sidebar.header("📊 Selection Summary")
        
        if st.session_state.selected_farm_id:
            selected_farm = next((f for f in st.session_state.farms if f['id'] == st.session_state.selected_farm_id), None)
            if selected_farm:
                st.sidebar.markdown(f"**Farm:** {selected_farm['name']}")
                st.sidebar.markdown(f"**ID:** {selected_farm['id']}")
                st.sidebar.markdown(f"**Location:** {selected_farm['latitude']:.6f}, {selected_farm['longitude']:.6f}")
        else:
            st.sidebar.info("No farm selected")
        
        st.sidebar.markdown("---")
        
        if st.session_state.selected_field_id:
            selected_field = next((f for f in st.session_state.fields if f['id'] == st.session_state.selected_field_id), None)
            if selected_field:
                st.sidebar.markdown(f"**Field:** {selected_field['name']}")
                st.sidebar.markdown(f"**ID:** {selected_field['id']}")
                st.sidebar.markdown(f"**Fenced:** {'Yes' if selected_field['fenced'] else 'No'}")
        else:
            st.sidebar.info("No field selected")
        
        st.sidebar.markdown("---")
        
        if st.session_state.selected_path_id:
            selected_path = next((p for p in st.session_state.paths if p['id'] == st.session_state.selected_path_id), None)
            if selected_path:
                st.sidebar.markdown(f"**Path:** {selected_path['name']}")
                st.sidebar.markdown(f"**ID:** {selected_path['id']}")
        else:
            st.sidebar.info("No path selected")
        
        st.sidebar.markdown("---")
        
        # Statistics
        st.sidebar.markdown("### 📈 Statistics")
        st.sidebar.markdown(f"**Total Farms:** {len(st.session_state.farms)}")
        st.sidebar.markdown(f"**Total Fields:** {len(st.session_state.fields)}")
        st.sidebar.markdown(f"**Total Paths:** {len(st.session_state.paths)}")
        
        if st.session_state.last_refresh_time:
            st.sidebar.markdown(f"**Last Refresh:** {st.session_state.last_refresh_time.strftime('%H:%M:%S')}")
            
    def run(self):
        """Main application loop."""
        # Initialize data
        if not st.session_state.farms:
            self.fetch_all_farms()
        
        # Render the application
        st.title("🚜 RISE SDVP - Farm, Field & Path Management")
        st.markdown("Web-based interface for managing farms, fields, and paths. Replicates the behavior of the Qt RControlStation tables.")
        
        # Create tabs
        tab1, tab2, tab3 = st.tabs(["🏠 Farms", "🌾 Fields", "🛣️ Paths"])
        
        with tab1:
            selected_farm_id = self.render_farm_table()
        
        with tab2:
            if st.session_state.selected_farm_id:
                selected_field_id = self.render_field_table(st.session_state.selected_farm_id)
            else:
                st.info("Please select a farm from the Farms tab first.")
                selected_field_id = None
        
        with tab3:
            if st.session_state.selected_field_id:
                selected_path_id = self.render_path_table(st.session_state.selected_field_id)
            else:
                st.info("Please select a field from the Fields tab first.")
                selected_path_id = None
        
        # Render summary in sidebar
        self.render_summary()
        
        # Add a global refresh button
        st.sidebar.markdown("---")
        if st.sidebar.button("🔄 Refresh All Data", width='stretch'):
            self.fetch_all_farms()
            if st.session_state.selected_farm_id:
                self.fetch_fields_for_farm(st.session_state.selected_farm_id)
            if st.session_state.selected_field_id:
                self.fetch_paths_for_field(st.session_state.selected_field_id)
            st.rerun()


def main():
    """Main entry point for the Streamlit application."""
    app = FarmManagerApp()
    app.run()


if __name__ == "__main__":
    main()