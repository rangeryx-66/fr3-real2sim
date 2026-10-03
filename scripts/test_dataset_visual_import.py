"""Regression checks for the visual-only Isaac converter compatibility copy."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from interactive_twin.visual_import import compatible_urdf, geometry_signature, repair, without_uv

OBJ = '''v 0 0 0
v 1 0 0
v 0 1 0
vt 0 0
vt 1 0
vt 0 1
vn 0 0 1
f 1/1/1 2/2/1 3/3/1
f -3/-3/1 -2/-2/1 -1/-1/1
'''


class VisualImportTests(unittest.TestCase):
    def test_exact_geometry_and_normals(self):
        fixed = without_uv(OBJ)
        self.assertEqual(geometry_signature(OBJ), geometry_signature(fixed))
        self.assertNotIn('vt ', fixed)
        self.assertIn('f 1//1 2//1 3//1', fixed)
        self.assertIn('f -3//1 -2//1 -1//1', fixed)

    def test_vertex_only_faces(self):
        text = 'v 0 0 0\nf 1 1 1\n'
        self.assertEqual(text, without_uv(text))

    def scene(self, folder):
        mesh = folder / 'part.obj'
        mesh.write_text(OBJ)
        collision = folder / 'collision.stl'
        collision.write_text('solid unchanged\nendsolid unchanged\n')
        urdf = folder / 'asset.urdf'
        urdf.write_text('''<robot name="dataset"><link name="root"/>
<link name="door"><visual><origin xyz="1 2 3" rpy=".1 .2 .3"/>
<geometry><mesh filename="part.obj" scale=".5 .5 .5"/></geometry></visual>
<collision><origin xyz=".1 .2 .3"/><geometry><mesh filename="collision.stl"/></geometry></collision>
<inertial><mass value="2"/><inertia ixx="1" iyy="2" izz="3"/></inertial></link>
<joint name="hinge" type="revolute"><parent link="root"/><child link="door"/>
<origin xyz="1 2 3"/><axis xyz="0 1 0"/><limit lower="0" upper="1" effort="0" velocity="1"/></joint></robot>''')
        return urdf

    def test_source_and_physics_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            source = self.scene(folder)
            originals = {str(p): p.read_bytes() for p in folder.iterdir()}
            audit = repair(source, folder / 'derived')
            for p, data in originals.items():
                self.assertEqual(Path(p).read_bytes(), data)
            a, b = ET.parse(source).getroot(), ET.parse(audit['derived_urdf']).getroot()
            self.assertEqual(ET.tostring(a.find('joint')), ET.tostring(b.find('joint')))
            self.assertEqual(ET.tostring(a.find('./link[@name="door"]/inertial')),
                             ET.tostring(b.find('./link[@name="door"]/inertial')))
            for tag in ('origin', 'geometry/mesh'):
                x = a.find('./link[@name="door"]/visual/' + tag).attrib.copy()
                y = b.find('./link[@name="door"]/visual/' + tag).attrib.copy()
                x.pop('filename', None); y.pop('filename', None)
                self.assertEqual(x, y)
            collision = b.find('./link[@name="door"]/collision/geometry/mesh')
            self.assertEqual(Path(collision.get('filename')), (folder / 'collision.stl').resolve())
            self.assertTrue(audit['physics_unchanged'])
            self.assertFalse(audit['texture_fidelity_preserved'])

    def test_cache_rejects_changed_collision(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            source = self.scene(folder)
            compatible_urdf(source, folder / 'cache')
            ((folder / 'collision.stl').resolve()).write_text('changed')
            with self.assertRaisesRegex(RuntimeError, 'COLLISION_CHANGED'):
                compatible_urdf(source, folder / 'cache')

    def test_cache_rejects_changed_visual(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            source = self.scene(folder)
            compatible_urdf(source, folder / 'cache')
            (folder / 'part.obj').write_text(OBJ + 'v 2 2 2\n')
            with self.assertRaisesRegex(RuntimeError, 'MESH_CHANGED'):
                compatible_urdf(source, folder / 'cache')


if __name__ == '__main__':
    unittest.main()
