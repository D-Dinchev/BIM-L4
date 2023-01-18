import NemAll_Python_Geometry as Geometry
import NemAll_Python_Reinforcement as Reinforcement
import NemAll_Python_BaseElements as BaseElements
import NemAll_Python_BasisElements as BasisElements
import NemAll_Python_Utility as Utility
import GeometryValidate as Validate

import StdReinfShapeBuilder.GeneralReinfShapeBuilder as GeneralShapeBuilder
import StdReinfShapeBuilder.LinearBarPlacementBuilder as LinearBarBuilder
from StdReinfShapeBuilder.ConcreteCoverProperties import ConcreteCoverProperties
from StdReinfShapeBuilder.ReinforcementShapeProperties import ReinforcementShapeProperties
from StdReinfShapeBuilder.RotationAngles import RotationAngles
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties
from HandleService import HandleService

print('Load Beam.py')

def check_allplan_version(build_ele, version):
    del build_ele
    del version
    return True


def create_element(build_ele, doc):
    element = BeamCreate(doc)
    return element.create(build_ele)


class BeamCreate:
    def __init__(self, doc):
        self.model_ele_list = []
        self.handle_list = []
        self.document = doc

        self.com_prop = BaseElements.CommonProperties()
        self.com_prop.GetGlobalProperties()

        self.length = None
        self.height = None
        self.width = None
        self.top_width = None
        self.top_height = None
        self.bottom_width = None
        self.bottom_height = None
        self.middle_width = None
        self.middle_height = None
        self.hole_depth = None
        self.hole_height = None
        self.angleX = Geometry.Angle()
        self.angleY = Geometry.Angle()
        self.angleZ = Geometry.Angle()
        self.concrete_grade = None
        self.steel_grade = None
        self.bar_diameter = None
        self.concrete_cover = None
        self.bar_distance = None
        self.bending_roller = None
        self.bar_height = None
        self.hook_length = None
        self.bar_depth = None

    def read_values(self, build_ele):
        self.length = build_ele.Length.value
        self.height = build_ele.Height.value

        self.top_width = build_ele.TopWidth.value
        self.top_height = build_ele.TopHeight.value

        self.bottom_width = build_ele.BottomWidth.value
        self.bottom_height = build_ele.BottomHeight.value

        self.width = max(self.top_width, self.bottom_width)

        self.middle_width = build_ele.MiddleWidth.value
        self.middle_height = build_ele.MiddleHeight.value

        self.hole_depth = build_ele.HoleDepth.value
        self.hole_height = build_ele.HoleHeight.value

        self.angleX = build_ele.AngleX.value
        self.angleY = build_ele.AngleY.value
        self.angleZ = build_ele.AngleZ.value

        self.concrete_grade = build_ele.ConcreteGrade.value
        self.steel_grade = build_ele.SteelGrade.value
        self.bar_diameter = build_ele.BarDiameter.value
        self.concrete_cover = build_ele.ConcreteCover.value
        self.bar_spacing = build_ele.BarSpacing.value
        self.bending_roller = build_ele.BendingRoller.value
        self.bar_height = build_ele.BarHeight.value
        self.hook_length = build_ele.HookLength.value
        self.bar_depth = build_ele.BarDepth.value

    def create(self, build_ele):
        self.read_values(build_ele)
        self.create_beam(build_ele)
        self.create_reinforcement(build_ele)
        self.create_handles(build_ele)
        BaseElements.ElementTransform(
            Geometry.Vector3D(),
            self.angleX,
            self.angleY,
            self.angleZ,
            self.model_ele_list,
        )
        rot_angles = RotationAngles(self.angleX, self.angleY, self.angleZ)
        HandleService.transform_handles(
            self.handle_list, rot_angles.get_rotation_matrix()
        )

        return (self.model_ele_list, self.handle_list)

    def create_beam(self, build_ele):
        beam_parts = Geometry.BRep3DList()
        # top part
        top = Geometry.BRep3D.CreateCuboid(
            Geometry.AxisPlacement3D(
                Geometry.Point3D(
                    (self.width - self.top_width) / 2.0,
                    0.0,
                    self.height - self.top_height,
                ),
                Geometry.Vector3D(1, 0, 0),
                Geometry.Vector3D(0, 0, 1),
            ),
            self.top_width / 2.0,
            self.length / 2.0,
            self.top_height,
        )
        for_extraction = Geometry.BRep3D.CreateCuboid(
            Geometry.AxisPlacement3D(
                Geometry.Point3D(
                    (self.width - self.top_width) / 2.0, 0.0, self.height - 45.0
                ),
                Geometry.Vector3D(1, 0, 0),
                Geometry.Vector3D(0, 0, 1),
            ),
            60.0,
            self.length / 2.0,
            45.0,
        )
        err, top = Geometry.MakeSubtraction(top, for_extraction)
        beam_parts.append(top)
        # middle part
        middle_part = Geometry.BRep3D.CreateCuboid(
            Geometry.AxisPlacement3D(
                Geometry.Point3D(0.0, 0.0, self.bottom_height),
                Geometry.Vector3D(1, 0, 0),
                Geometry.Vector3D(0, 0, 1),
            ),
            self.width / 2.0,
            self.length / 2.0,
            self.middle_height,
        )
        beam_parts.append(middle_part)
        # bottom part
        bottom = Geometry.BRep3D.CreateCuboid(
            Geometry.AxisPlacement3D(
                Geometry.Point3D((self.width - self.bottom_width) / 2.0, 0.0, 0.0),
                Geometry.Vector3D(1, 0, 0),
                Geometry.Vector3D(0, 0, 1),
            ),
            self.bottom_width / 2.0,
            self.length / 2.0,
            self.bottom_height,
        )
        edges = Utility.VecSizeTList()
        edges.append(10)
        err, bottom = Geometry.ChamferCalculus.Calculate(bottom, edges, 20.0, False)
        beam_parts.append(bottom)
        # beam union
        err, beam = Geometry.MakeUnion(beam_parts)

        # parts for extraction
        extraction_parts = Geometry.BRep3DList()
        for_extraction_pol = Geometry.Polyline3D()
        start_point = Geometry.Point3D(
            (self.width - self.middle_width) / 2, 0, self.height - self.top_height
        )
        for_extraction_pol += start_point
        for_extraction_pol += Geometry.Point3D(
            (self.width - self.middle_width) / 2, 0, self.bottom_height
        )
        for_extraction_pol += Geometry.Point3D(
            (self.width - self.bottom_width) / 2, 0, 153
        )
        for_extraction_pol += Geometry.Point3D(-10, 0, 153)
        for_extraction_pol += Geometry.Point3D(-10, 0, self.height - 100)
        for_extraction_pol += Geometry.Point3D(
            (self.width - self.top_width) / 2, 0.0, self.height - 100
        )
        for_extraction_pol += start_point
        path = Geometry.Polyline3D()
        path += Geometry.Point3D(0, 0, 0)
        path += Geometry.Point3D(0, self.length / 2, 0)

        err, for_extraction = Geometry.CreateSweptBRep3D(
            for_extraction_pol, path, False, None
        )
        edges = Utility.VecSizeTList()
        edges.append(3)
        edges.append(1)
        err, for_extraction = Geometry.FilletCalculus3D.Calculate(
            for_extraction, edges, 100, False
        )
        extraction_parts.append(for_extraction)

        # holes
        hole = Geometry.BRep3D.CreateCylinder(
            Geometry.AxisPlacement3D(
                Geometry.Point3D(
                    0, build_ele.HoleDepth.value, build_ele.HoleHeight.value
                ),
                Geometry.Vector3D(0, 0, 1),
                Geometry.Vector3D(1, 0, 0),
            ),
            45.5,
            self.width,
        )
        extraction_parts.append(hole)
        # hole extraction
        err, beam = Geometry.MakeSubtraction(beam, extraction_parts)

        # beam
        plane = Geometry.Plane3D(
            Geometry.Point3D(self.width / 2, 0, 0), Geometry.Vector3D(1, 0, 0)
        )
        err, beam = Geometry.MakeUnion(beam, Geometry.Mirror(beam, plane))
        plane.Set(Geometry.Point3D(0, self.length / 2, 0), Geometry.Vector3D(0, 1, 0))
        err, beam = Geometry.MakeUnion(beam, Geometry.Mirror(beam, plane))
        self.model_ele_list.append(BasisElements.ModelElement3D(self.com_prop, beam))

    def create_reinforcement(self, build_ele):
        line1 = Geometry.Line3D(
            Geometry.Point3D(
                (self.width - self.middle_width) / 2, 0, self.height - self.top_height
            ),
            Geometry.Point3D((self.width - self.top_width) / 2, 0, self.height - 100),
        )
        x = (self.width - self.top_width) / 2 + 60 + self.concrete_cover
        line2 = Geometry.Line3D(
            Geometry.Point3D(x, 0, self.height - self.top_height),
            Geometry.Point3D(x, 0, self.height),
        )
        err, intersection = Geometry.IntersectionCalculus(line1, line2)
        if self.height - self.bar_depth < intersection.Z:
            build_ele.BarDepth.value = self.height - intersection.Z
            self.bar_depth = build_ele.BarDepth.value
        if self.height - self.bar_depth < intersection.Z:
            build_ele.BarDepth.value = self.height - intersection.Z
            self.bar_depth = build_ele.BarDepth.value

        model_angles = RotationAngles(90, -90, 0)

        shape_props = ReinforcementShapeProperties.rebar(
            self.bar_diameter,
            self.bending_roller,
            self.steel_grade,
            self.concrete_grade,
            Reinforcement.BendingShapeType.LongitudinalBar,
        )
        concrete_cover_props = ConcreteCoverProperties.left_right_bottom(
            self.concrete_cover, self.concrete_cover, -self.concrete_cover
        )

        shape = GeneralShapeBuilder.create_longitudinal_shape_with_hooks(
            self.bar_height + self.bar_depth,
            model_angles,
            shape_props,
            concrete_cover_props,
            -1,
            self.hook_length,
        )

        x = (self.width - self.top_width) / 2 + 60
        z = self.height - self.bar_depth
        from_point = Geometry.Point3D(x, 0, z)
        to_point = Geometry.Point3D(x, self.length, z)
        self.model_ele_list.append(
            LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(
                0, shape, from_point, to_point, 0, 0, self.bar_spacing
            )
        )

        shape.Rotate(RotationAngles(0, 0, 180))
        x = self.top_width - 60.0
        from_point = Geometry.Point3D(x, 0, z)
        to_point = Geometry.Point3D(x, self.length, z)
        self.model_ele_list.append(
            LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(
                0, shape, from_point, to_point, 0, 0, self.bar_spacing
            )
        )

    def create_handles(self, build_ele):
        handle1 = HandleProperties(
            "Length",
            Geometry.Point3D(0, self.length, 0),
            Geometry.Point3D(0, 0, 0),
            [("Length", HandleDirection.point_dir)],
            HandleDirection.point_dir,
            True,
        )
        self.handle_list.append(handle1)
        handle2 = HandleProperties(
            "Height",
            Geometry.Point3D(0, 0, self.height),
            Geometry.Point3D(0, 0, 0),
            [("Height", HandleDirection.point_dir)],
            HandleDirection.point_dir,
            True,
        )
        self.handle_list.append(handle2)
        handle3 = HandleProperties(
            "TopWidth",
            Geometry.Point3D(
                (self.width - self.top_width) / 2 + self.top_width,
                0.0,
                self.height - 45,
            ),
            Geometry.Point3D((self.width - self.top_width) / 2, 0, self.height - 45),
            [("TopWidth", HandleDirection.point_dir)],
            HandleDirection.point_dir,
            True,
        )
        self.handle_list.append(handle3)
        handle4 = HandleProperties(
            "MiddleWidth",
            Geometry.Point3D(
                (self.width - self.middle_width) / 2 + self.middle_width,
                0.0,
                self.height / 2,
            ),
            Geometry.Point3D((self.width - self.middle_width) / 2, 0, self.height / 2),
            [("MiddleWidth", HandleDirection.point_dir)],
            HandleDirection.point_dir,
            True,
        )
        self.handle_list.append(handle4)
        handle5 = HandleProperties(
            "BottomWidth",
            Geometry.Point3D(
                (self.width - self.bottom_width) / 2 + self.bottom_width, 0, 153
            ),
            Geometry.Point3D((self.width - self.bottom_width) / 2, 0, 153),
            [("BottomWidth", HandleDirection.point_dir)],
            HandleDirection.point_dir,
            True,
        )
        self.handle_list.append(handle5)
    
        

        