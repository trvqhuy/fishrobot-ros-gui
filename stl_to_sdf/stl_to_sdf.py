import os
from stl import mesh
from lxml import etree

def stl_to_sdf(stl_file, sdf_file, model_name="example_model"):
    # Read the STL file
    your_mesh = mesh.Mesh.from_file(stl_file)

    # Create the SDF XML structure
    sdf = etree.Element("sdf", version="1.6")
    model = etree.SubElement(sdf, "model", name=model_name)
    static = etree.SubElement(model, "static")
    static.text = "true"
    link = etree.SubElement(model, "link", name="link")

    # Add visual element
    visual = etree.SubElement(link, "visual", name="visual")
    geometry = etree.SubElement(visual, "geometry")
    mesh_elem = etree.SubElement(geometry, "mesh")
    uri = etree.SubElement(mesh_elem, "uri")
    uri.text = f"model://{model_name}/meshes/{os.path.basename(stl_file)}"

    # Add collision element
    collision = etree.SubElement(link, "collision", name="collision")
    geometry = etree.SubElement(collision, "geometry")
    mesh_elem = etree.SubElement(geometry, "mesh")
    uri = etree.SubElement(mesh_elem, "uri")
    uri.text = f"model://{model_name}/meshes/{os.path.basename(stl_file)}"

    # Write the SDF file
    tree = etree.ElementTree(sdf)
    tree.write(sdf_file, pretty_print=True, xml_declaration=True, encoding="UTF-8")

    # Create model.config file
    config_file = os.path.join(os.path.dirname(sdf_file), 'model.config')
    with open(config_file, 'w') as f:
        f.write(
            f"""<?xml version="1.0" ?>
                <model>
                <name>{model_name}</name>
                <version>1.0</version>
                <sdf version="1.6">model.sdf</sdf>
                <author>
                    <name>Your Name</name>
                    <email>your.email@example.com</email>
                </author>
                <description>
                    A description of your model.
                </description>
                </model>"""
        )

if __name__ == "__main__":
    # Example usage
    stl_file = "input/Body-V1.STL"
    sdf_file = "Body-V1.sdf"
    
    # # Ensure the output directory exists
    # os.makedirs(os.path.dirname(sdf_file), exist_ok=True)
    
    # # Copy the STL file to the meshes directory
    # os.makedirs(os.path.join(os.path.dirname(sdf_file), "meshes"), exist_ok=True)
    # os.system(f"cp {stl_file} {os.path.join(os.path.dirname(sdf_file), 'meshes')}")
    
    stl_to_sdf(stl_file, sdf_file)
