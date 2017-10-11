# Add "transmission" elements to robot model URDF

import xml.etree.ElementTree as ET
from xml.dom import minidom
import re

urdf_path = "simplified_human_model.urdf"


def prettify(elem):
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    pretty = reparsed.toprettyxml(indent="  ")
    return pretty

def main():
    et = ET.parse(urdf_path)
    root = et.getroot()
    elements = [child for child in root.getchildren()]

    # Add transmission elements
    joints = [e for e in elements if e.tag=="joint" and e.attrib["type"]=="continuous" ]
    for j in joints:
        t = ET.SubElement(root, "transmission")
        t.attrib["name"] = j.attrib["name"] + "_transmission"

	typ = ET.SubElement(t, "type")
	typ.text = "transmission_interface/SimpleTransmission"

        jname = ET.SubElement(t, "joint")
	jname.attrib["name"] = j.attrib["name"]

        actuator = ET.SubElement(t, "actuator")
        actuator.attrib["name"] = j.attrib["name"] + "_actuator"

        mr = ET.SubElement(actuator, "mechanicalReduction")
	mr.text = "1"
        
        #type = ET.SubElement(t, "type")

    # TODO Better way to remove the extra blank lines
    tree = prettify(root)
    g = open("temp.urdf", 'w+')
    g.write(tree)
    g.close()
    g = open("temp.urdf", "r")
    f = open("new_human_model.urdf", 'w+')
    for line in g.readlines():
	not_blank = (len(line)-line.count(' ')-line.count('\n'))
        if not_blank>0:
	    f.write(line)
    f.close()
 
if __name__ == "__main__":
    main()
