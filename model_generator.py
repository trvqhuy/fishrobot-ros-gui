import xml.etree.ElementTree as ET
import xml.dom.minidom
import ruamel.yaml
import json
import os

YAML_DIRECTORY = "config/foundation.bridge.yaml"
YAML_OUTPUT_DIRECTORY = "config/final.bridge.yaml"
FOUNDATION_DIRECTORY = "config/foundation.world.sdf"
FIN_MODEL_DIRECTORY = "config/foundation.fin_model.sdf"
OUTPUT_DIRECTORY = "config/final.fish_world.sdf"

WORLD_NAME = "fish_world"
FISH_MODEL_NAME = "fish_model"

CONFIG_FILE = "config/gui_config.json"

# Load GUI parameters
if not os.path.exists(CONFIG_FILE):
    raise FileNotFoundError(f"Missing GUI config file: {CONFIG_FILE}")

with open(CONFIG_FILE, "r") as f:
    gui_config = json.load(f)

# Override parameters from config
LINK_NUMBER = int(gui_config.get("link_number", 50))
FISH_LENGTH = float(gui_config.get("fish_length", 1.220))
FISH_WIDTH = float(gui_config.get("fish_width", 0.360))
MEMBRANCE_WIDTH = float(gui_config.get("membrance_width", 0.170))
MEMBRANCE_LENGTH = float(gui_config.get("membrance_length", 1.096))
MEMBRANE_1_ON = gui_config.get("membrane_1_on", True)
MEMBRANE_2_ON = gui_config.get("membrane_2_on", True)

# FISH_LENGTH = 1.220     #in meters
# FISH_WIDTH = 0.360
# LINK_NUMBER = 50
# FISH_CENTER_OFFSET = 0.085
# MEMBRANCE_WIDTH = 0.170
# MEMBRANCE_LENGTH = 1.096
# MEMBRANE_1_ON = True
# MEMBRANE_2_ON = True

FIN_SIZE = [0.005, MEMBRANCE_WIDTH, MEMBRANCE_LENGTH*2/LINK_NUMBER]
FIN_CIRCULAR_SIZE = [FIN_SIZE[2]/2.0, 0.01]


xml_tree = ET.parse(FOUNDATION_DIRECTORY)
root = xml_tree.getroot()
world_tree = root.find('world')

# Change World Name
world_tree.set('name', WORLD_NAME)

# Fish Model
fish_model = world_tree.find('model')
fish_model.set('name', FISH_MODEL_NAME)

for iter, each_item in enumerate(range(LINK_NUMBER)):
    fin_mdel_xml = ET.parse(FIN_MODEL_DIRECTORY).getroot()

    # Fin Link Configurations
    fin_link = fin_mdel_xml.find('link')

    fin_link.set('name', 'fin_1_%d' % (iter + 1))
    for each_child in fin_link:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_1_%d_circular' % (iter + 1)
    fin_link.find('.//pose').text = (
        "0.0 0.0 -%f 3.14 0 0" % (0.2)
    )
    fin_link.find('.//inertial/mass').text = (
        '%f' % (FIN_SIZE[0] * FIN_SIZE[1] * FIN_SIZE[2] * 1000.0)
    )
    fin_link.find('.//visual/geometry/box/size').text = (
        '%f %f %f' % (FIN_SIZE[0], FIN_SIZE[2], FIN_SIZE[1], )
    )
    fin_link.find('.//collision/geometry/box/size').text = (
        '%f %f %f' % (FIN_SIZE[0], FIN_SIZE[2], FIN_SIZE[1], )
    )

    fin_joint = fin_mdel_xml.find('joint')
    for each_child in fin_joint:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_1_%d_circular' % (iter + 1)

    fin_joint.set('name', 'fin_1_%d_joint' % (iter + 1))
    fin_joint.find('child').text = 'fin_1_%d' % (iter + 1)
    fin_joint.find('parent').text = 'fin_1_%d_circular' % (iter + 1)

    fin_plugin = fin_mdel_xml.find('plugin')
    fin_plugin.find('.//joint_name').text = (
        'fin_1_%d_joint' % (iter + 1)
    )

    fin_dynamic = fin_mdel_xml.findall('plugin')[-1]
    fin_dynamic.find('link_name').text = 'fin_1_%d' % (iter + 1)
    fin_dynamic.find('cp').text ='0 0 0' #% (FIN_SIZE[1]/2)
    fin_dynamic.find('upward').text = '1.0 0 0'
    fin_dynamic.find('forward').text = '0 1.0 0'


    # fin_dynamic = fin_mdel_xml.findall('plugin')[-1]
    # fin_dynamic.find('link_name').text = 'fin_2_%d' % (iter + 1)
    # fin_dynamic.find('cp').text = '0 %.2f 0' % (FIN_SIZE[1]) 
    # fin_dynamic.find('upward').text = '0 0 0'
    # fin_dynamic.find('forward').text = '0 0 0'

    fin_circular_link = fin_mdel_xml.findall('link')[1]
    for each_child in fin_circular_link:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_1_%d_axis' % (iter + 1)
    fin_circular_link.set('name', 'fin_1_%d_circular' % (iter + 1))
    distance = MEMBRANCE_LENGTH/2 - (iter + 1/2) * (MEMBRANCE_LENGTH/LINK_NUMBER)
    fin_circular_link.find('.//pose').text = (
        "0.0 0.0 -%f 0 0 0" % (FIN_CIRCULAR_SIZE[1])
    )
    fin_circular_link.find('.//inertial/mass').text = (
        '%f' % (3.1416 * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[1] * 1000.0)
    )
    fin_circular_link.find('.//visual/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_circular_link.find('.//visual/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )
    fin_circular_link.find('.//collision/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_circular_link.find('.//collision/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )

    fin_circular_joint = fin_mdel_xml.findall('joint')[1]
    for each_child in fin_circular_joint:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_1_%d_axis' % (iter + 1)

    fin_circular_joint.set('name', 'fin_1_%d_circular_joint' % (iter + 1))
    fin_circular_joint.find('child').text = 'fin_1_%d_circular' % (iter + 1)
    fin_circular_joint.find('parent').text = 'fin_1_%d_axis' % (iter + 1)

    fin_circular_plugin = fin_mdel_xml.findall('plugin')[1]
    fin_circular_plugin.find('.//joint_name').text = (
        'fin_1_%d_circular_joint' % (iter + 1)
    )

    fin_fix_link = fin_mdel_xml.findall('link')[2]
    for each_child in fin_fix_link:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fish_body'
    fin_fix_link.set('name', 'fin_1_%d_axis' % (iter + 1))
    distance = MEMBRANCE_LENGTH - (iter + 1/2) * (MEMBRANCE_LENGTH/LINK_NUMBER)
    # fin_fix_link.find('.//pose').text = (
    #     "0.0 %f %f 1.5708 0 1.5708" % (0, distance)
    # )

    fin_fix_link.find('.//pose').text = (
        "%f %f %f 1.5708 0 1.5708" %
        (-FISH_WIDTH/2+0.05, 0, -(MEMBRANCE_LENGTH/2)+distance)
    )
    fin_fix_link.find('.//inertial/mass').text = (
        '%f' % (3.1416 * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[1] * 1000.0)
    )
    fin_fix_link.find('.//visual/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_fix_link.find('.//visual/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )
    fin_fix_link.find('.//collision/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_fix_link.find('.//collision/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )

    fin_fix_joint = fin_mdel_xml.findall('joint')[2]
    for each_child in fin_fix_joint:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fish_body'

    fin_fix_joint.set('name', 'fin_1_%d_axis_joint' % (iter + 1))
    fin_fix_joint.find('child').text = 'fin_1_%d_axis' % (iter + 1)
    fin_fix_joint.find('parent').text = 'fish_body'

    if MEMBRANE_1_ON:
        fish_model.append(fin_link)
        fish_model.append(fin_joint)
        fish_model.append(fin_plugin)

        fish_model.append(fin_circular_link)
        fish_model.append(fin_circular_joint)
        fish_model.append(fin_circular_plugin)

        fish_model.append(fin_fix_link)
        fish_model.append(fin_fix_joint)

        fish_model.append(fin_dynamic)


for iter, each_item in enumerate(range(LINK_NUMBER)):
    fin_mdel_xml = ET.parse(FIN_MODEL_DIRECTORY).getroot()

    # Fin Link Configurations
    fin_link = fin_mdel_xml.find('link')

    fin_link.set('name', 'fin_2_%d' % (iter + 1))
    for each_child in fin_link:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_2_%d_circular' % (iter + 1)

    fin_link.find('.//pose').text = (
        "0.0 0.0 %f 0 0 0" % (0.2)
    )
    fin_link.find('.//inertial/mass').text = (
        '%f' % (FIN_SIZE[0] * FIN_SIZE[1] * FIN_SIZE[2] * 1000.0)
    )
    fin_link.find('.//visual/geometry/box/size').text = (
        '%f %f %f' % (FIN_SIZE[0], FIN_SIZE[2], FIN_SIZE[1], )
    )
    fin_link.find('.//collision/geometry/box/size').text = (
        '%f %f %f' % (FIN_SIZE[0], FIN_SIZE[2], FIN_SIZE[1], )
    )

    fin_joint = fin_mdel_xml.find('joint')
    for each_child in fin_joint:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_2_%d_circular' % (iter + 1)

    fin_joint.set('name', 'fin_2_%d_joint' % (iter + 1))
    fin_joint.find('child').text = 'fin_2_%d' % (iter + 1)
    fin_joint.find('parent').text = 'fin_2_%d_circular' % (iter + 1)

    fin_plugin = fin_mdel_xml.find('plugin')
    fin_plugin.find('.//joint_name').text = (
        'fin_2_%d_joint' % (iter + 1)
    )

    fin_dynamic = fin_mdel_xml.findall('plugin')[-1]
    fin_dynamic.find('link_name').text = 'fin_2_%d' % (iter + 1)
    fin_dynamic.find('cp').text ='0 0 0' #% (FIN_SIZE[1]/2)
    fin_dynamic.find('upward').text = '1.0 0 0'
    fin_dynamic.find('forward').text = '0 1.0 0'

    fin_circular_link = fin_mdel_xml.findall('link')[1]
    for each_child in fin_circular_link:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_2_%d_axis' % (iter + 1)
    fin_circular_link.set('name', 'fin_2_%d_circular' % (iter + 1))
    distance = MEMBRANCE_LENGTH/2 - (iter + 1/2) * (MEMBRANCE_LENGTH/LINK_NUMBER)
    fin_circular_link.find('.//pose').text = (
        "0.0 0.0 %f 0 0 0" % (FIN_CIRCULAR_SIZE[1])
    )
    fin_circular_link.find('.//inertial/mass').text = (
        '%f' % (3.1416 * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[1] * 1000.0)
    )
    fin_circular_link.find('.//visual/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_circular_link.find('.//visual/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )
    fin_circular_link.find('.//collision/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_circular_link.find('.//collision/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )

    fin_circular_joint = fin_mdel_xml.findall('joint')[1]
    for each_child in fin_circular_joint:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fin_2_%d_axis' % (iter + 1)

    fin_circular_joint.set('name', 'fin_2_%d_circular_joint' % (iter + 1))
    fin_circular_joint.find('child').text = 'fin_2_%d_circular' % (iter + 1)
    fin_circular_joint.find('parent').text = 'fin_2_%d_axis' % (iter + 1)

    fin_circular_plugin = fin_mdel_xml.findall('plugin')[1]
    fin_circular_plugin.find('.//joint_name').text = (
        'fin_2_%d_circular_joint' % (iter + 1)
    )


    fin_fix_link = fin_mdel_xml.findall('link')[2]
    for each_child in fin_fix_link:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fish_body'
    fin_fix_link.set('name', 'fin_2_%d_axis' % (iter + 1))
    distance = MEMBRANCE_LENGTH - (iter + 1/2) * (MEMBRANCE_LENGTH/LINK_NUMBER)

    fin_fix_link.find('.//pose').text = (
        "%f %f %f 1.5708 0 1.5708" %
        (FISH_WIDTH/2-0.05, 0, -(MEMBRANCE_LENGTH/2)+distance)
    )
    fin_fix_link.find('.//inertial/mass').text = (
        '%f' % (3.1416 * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[0] * FIN_CIRCULAR_SIZE[1] * 1000.0)
    )
    fin_fix_link.find('.//visual/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_fix_link.find('.//visual/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )
    fin_fix_link.find('.//collision/geometry/cylinder/radius').text = (
        '%f' % (FIN_CIRCULAR_SIZE[0])
    )
    fin_fix_link.find('.//collision/geometry/cylinder/length').text = (
        '%f' % (FIN_CIRCULAR_SIZE[1])
    )

    fin_fix_joint = fin_mdel_xml.findall('joint')[2]
    for each_child in fin_fix_joint:
        if each_child.tag == 'pose':
            each_child.attrib['relative_to'] = 'fish_body'

    fin_fix_joint.set('name', 'fin_2_%d_axis_joint' % (iter + 1))
    fin_fix_joint.find('child').text = 'fin_2_%d_axis' % (iter + 1)
    fin_fix_joint.find('parent').text = 'fish_body'

    if MEMBRANE_2_ON:
        fish_model.append(fin_link)
        fish_model.append(fin_joint)
        fish_model.append(fin_plugin)

        fish_model.append(fin_circular_link)
        fish_model.append(fin_circular_joint)
        fish_model.append(fin_circular_plugin)

        fish_model.append(fin_fix_link)
        fish_model.append(fin_fix_joint)

        fish_model.append(fin_dynamic)
    

# Beautify the XML file
xml_content = ET.tostring(root, encoding="utf-8")
dom = xml.dom.minidom.parseString(xml_content)
pretty_xml = dom.toprettyxml(indent="\t", newl="\n")

# Save the beautified XML to a file
with open(OUTPUT_DIRECTORY, "w") as file:
    file.write(pretty_xml)



yaml = ruamel.yaml.YAML()
# yaml.preserve_quotes = True
with open(YAML_DIRECTORY) as fp:
    data = yaml.load(fp)

output_data = []

for iter in range(LINK_NUMBER):
    output_data.append({
        'ros_topic_name': data[0]['ros_topic_name'],
        'gz_topic_name': data[0]['gz_topic_name'],
        'ros_type_name': data[0]['ros_type_name'],
        'gz_type_name': data[0]['gz_type_name'],
    })
    output_data[-1]['ros_topic_name'] = (
        '/joints/fin_1_%d/cmd_pos' % (iter + 1)
    )

    output_data[-1]['gz_topic_name'] = (
        '/model/' + FISH_MODEL_NAME 
        + '/joint/fin_1_%d_joint/0/cmd_pos' % (iter + 1)
    )

    output_data.append({
        'ros_topic_name': data[1]['ros_topic_name'],
        'gz_topic_name': data[1]['gz_topic_name'],
        'ros_type_name': data[1]['ros_type_name'],
        'gz_type_name': data[1]['gz_type_name'],
    })
    output_data[-1]['ros_topic_name'] = (
        '/joints/fin_1_%d_circular/cmd_pos' % (iter + 1)
    )

    output_data[-1]['gz_topic_name'] = (
        '/model/' + FISH_MODEL_NAME 
        + '/joint/fin_1_%d_circular_joint/0/cmd_pos' % (iter + 1)
    )


for iter in range(LINK_NUMBER):
    output_data.append({
        'ros_topic_name': data[0]['ros_topic_name'],
        'gz_topic_name': data[0]['gz_topic_name'],
        'ros_type_name': data[0]['ros_type_name'],
        'gz_type_name': data[0]['gz_type_name'],
    })
    output_data[-1]['ros_topic_name'] = (
        '/joints/fin_2_%d/cmd_pos' % (iter + 1)
    )

    output_data[-1]['gz_topic_name'] = (
        '/model/' + FISH_MODEL_NAME 
        + '/joint/fin_2_%d_joint/0/cmd_pos' % (iter + 1)
    )

    output_data.append({
        'ros_topic_name': data[1]['ros_topic_name'],
        'gz_topic_name': data[1]['gz_topic_name'],
        'ros_type_name': data[1]['ros_type_name'],
        'gz_type_name': data[1]['gz_type_name'],
    })
    output_data[-1]['ros_topic_name'] = (
        '/joints/fin_2_%d_circular/cmd_pos' % (iter + 1)
    )

    output_data[-1]['gz_topic_name'] = (
        '/model/' + FISH_MODEL_NAME 
        + '/joint/fin_2_%d_circular_joint/0/cmd_pos' % (iter + 1)
    )

output_data.append({
    'ros_topic_name': '/imu',
    'gz_topic_name': '/imu',
    'ros_type_name': 'sensor_msgs/msg/Imu',
    'gz_type_name': 'gz.msgs.IMU',
})

output_data.append({
    'ros_topic_name': '/world/fish_world/dynamic_pose/info',
    'gz_topic_name': '/world/fish_world/dynamic_pose/info',
    'ros_type_name': 'geometry_msgs/msg/PoseArray',
    'gz_type_name': 'gz.msgs.Pose_V',
})

with open(YAML_OUTPUT_DIRECTORY, "w") as f:
    yaml.dump(output_data, f)

# xml_tree.write(
#     OUTPUT_DIRECTORY,
#     xml_declaration=True,
#     encoding="UTF-8")