#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import sys

def add_farm_table_properly():
    # Parse the XML file
    tree = ET.parse('mainwindow.ui')
    root = tree.getroot()
    
    # Find the log tab (tabAnalysis)
    log_tab = None
    for widget in root.findall('.//widget'):
        title = widget.find("./attribute[@name='title']/string")
        if title is not None and title.text == "Log":
            log_tab = widget
            break
    
    if log_tab is None:
        print("Could not find log tab")
        return False
    
    # Find the vBoxLayoutMain
    vbox_main = None
    for layout in log_tab.findall('.//layout[@name="vBoxLayoutMain"]'):
        vbox_main = layout
        break
    
    if vbox_main is None:
        print("Could not find vBoxLayoutMain")
        return False
    
    # Create a new horizontal layout for the table and existing content
    new_hbox = ET.Element('layout', {'class': 'QHBoxLayout', 'name': 'logContentLayout'})
    
    # Add the new farm table as the first item
    table_item = ET.SubElement(new_hbox, 'item')
    new_table = ET.SubElement(table_item, 'widget', {'class': 'QTableView', 'name': 'logFarmTable'})
    
    # Copy properties from farmTable structure
    size_policy = ET.SubElement(new_table, 'property', {'name': 'sizePolicy'})
    sizepolicy_elem = ET.SubElement(size_policy, 'sizepolicy', {'hsizetype': 'Expanding', 'vsizetype': 'Expanding'})
    ET.SubElement(sizepolicy_elem, 'horstretch').text = '1'
    ET.SubElement(sizepolicy_elem, 'verstretch').text = '1'
    
    min_size = ET.SubElement(new_table, 'property', {'name': 'minimumSize'})
    size_elem = ET.SubElement(min_size, 'size')
    ET.SubElement(size_elem, 'width').text = '400'
    ET.SubElement(size_elem, 'height').text = '300'
    
    selection_behavior = ET.SubElement(new_table, 'property', {'name': 'selectionBehavior'})
    enum_elem = ET.SubElement(selection_behavior, 'enum')
    enum_elem.text = 'QAbstractItemView::SelectionBehavior::SelectRows'
    
    # Move existing hBoxDisplay into the new layout
    hbox_display = None
    for item in vbox_main.findall('item'):
        layout = item.find('layout[@name="hBoxDisplay"]')
        if layout is not None:
            hbox_display = item
            break
    
    if hbox_display is not None:
        # Add the existing content as the second item
        new_hbox.append(hbox_display)
        # Remove from vbox_main
        vbox_main.remove(hbox_display)
    
    # Add the new layout to vbox_main
    new_item = ET.SubElement(vbox_main, 'item')
    new_item.append(new_hbox)
    
    # Write the modified XML back to the file
    tree.write('mainwindow.ui', encoding='utf-8', xml_declaration=True)
    
    print("Successfully added farm table to log tab")
    return True

if __name__ == '__main__':
    success = add_farm_table_properly()
    sys.exit(0 if success else 1)