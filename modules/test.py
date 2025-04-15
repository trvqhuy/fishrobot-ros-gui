import xml.etree.ElementTree as ET

xml_string = '''
<joint name='fin_1_1_joint' type='revolute'>
    <pose relative_to='fin_1_axis'/>
    <child>fin_1_1</child>
    <parent>fin_1_axis</parent>
    <axis>
        <xyz>0 0 1</xyz>
        <limit>
            <lower>-2.08567</lower>
            <upper>2.08567</upper>
            <effort>40.25</effort>
            <velocity>50</velocity>
        </limit>
        <dynamics>
            <damping>1</damping>
            <friction>0</friction>
            <spring_reference>0</spring_reference>
            <spring_stiffness>0</spring_stiffness>
        </dynamics>
    </axis>
</joint>
'''

# Parse the XML string
root = ET.fromstring(xml_string)

# Find the 'pose' element
pose_element = root.find('.//pose')

# Update the 'relative_to' attribute
pose_element.set('relative_to', 'new_relative_to')

# Convert the modified XML back to a string
modified_xml_string = ET.tostring(root, encoding='utf-8').decode('utf-8')

print(modified_xml_string)