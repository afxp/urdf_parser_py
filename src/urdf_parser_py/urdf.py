from urdf_parser_py.xml_reflection.basics import *
import urdf_parser_py.xml_reflection as xmlr

# Add a 'namespace' for names to avoid a conflict between URDF and SDF?
# A type registry? How to scope that? Just make a 'global' type pointer?
# Or just qualify names? urdf.geometric, sdf.geometric

xmlr.start_namespace('urdf')

xmlr.add_type('element_link', xmlr.SimpleElementType('link', str))
xmlr.add_type('element_xyz', xmlr.SimpleElementType('xyz', 'vector3'))

verbose = True


class Pose(xmlr.Object):
    def __init__(self, xyz=None, rpy=None):
        self.xyz = xyz
        self.rpy = rpy

    def check_valid(self):
        assert (self.xyz is None or len(self.xyz) == 3) and \
            (self.rpy is None or len(self.rpy) == 3)

    # Aliases for backwards compatibility
    @property
    def rotation(self): return self.rpy

    @rotation.setter
    def rotation(self, value): self.rpy = value

    @property
    def position(self): return self.xyz

    @position.setter
    def position(self, value): self.xyz = value


xmlr.reflect(Pose, tag='origin', params=[
    xmlr.Attribute('xyz', 'vector3', False, default=[0, 0, 0]),
    xmlr.Attribute('rpy', 'vector3', False, default=[0, 0, 0])
])


# Common stuff
name_attribute = xmlr.Attribute('name', str)
origin_element = xmlr.Element('origin', Pose, False)


class Color(xmlr.Object):
    def __init__(self, *args):
        # What about named colors?
        count = len(args)
        if count == 4 or count == 3:
            self.rgba = args
        elif count == 1:
            self.rgba = args[0]
        elif count == 0:
            self.rgba = None
        if self.rgba is not None:
            if len(self.rgba) == 3:
                self.rgba += [1.]
            if len(self.rgba) != 4:
                raise Exception('Invalid color argument count')


xmlr.reflect(Color, tag='color', params=[
    xmlr.Attribute('rgba', 'vector4')
])


class JointDynamics(xmlr.Object):
    def __init__(self, damping=None, friction=None):
        self.damping = damping
        self.friction = friction


xmlr.reflect(JointDynamics, tag='dynamics', params=[
    xmlr.Attribute('damping', float, False),
    xmlr.Attribute('friction', float, False)
])


class Box(xmlr.Object):
    def __init__(self, size=None):
        self.size = size


xmlr.reflect(Box, tag='box', params=[
    xmlr.Attribute('size', 'vector3')
])


class Cylinder(xmlr.Object):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length


xmlr.reflect(Cylinder, tag='cylinder', params=[
    xmlr.Attribute('radius', float),
    xmlr.Attribute('length', float)
])


class Sphere(xmlr.Object):
    def __init__(self, radius=0.0):
        self.radius = radius


xmlr.reflect(Sphere, tag='sphere', params=[
    xmlr.Attribute('radius', float)
])


class Mesh(xmlr.Object):
    def __init__(self, filename=None, scale=None):
        self.filename = filename
        self.scale = scale


xmlr.reflect(Mesh, tag='mesh', params=[
    xmlr.Attribute('filename', str),
    xmlr.Attribute('scale', 'vector3', required=False)
])


class GeometricType(xmlr.ValueType):
    def __init__(self):
        self.factory = xmlr.FactoryType('geometric', {
            'box': Box,
            'cylinder': Cylinder,
            'sphere': Sphere,
            'mesh': Mesh
        })

    def from_xml(self, node, path):
        children = xml_children(node)
        assert len(children) == 1, 'One element only for geometric'
        return self.factory.from_xml(children[0], path=path)

    def write_xml(self, node, obj):
        name = self.factory.get_name(obj)
        child = node_add(node, name)
        obj.write_xml(child)


xmlr.add_type('geometric', GeometricType())


class Collision(xmlr.Object):
    def __init__(self, geometry=None, origin=None, name=None):
        self.geometry = geometry
        self.name = name
        self.origin = origin


xmlr.reflect(Collision, tag='collision', params=[
    xmlr.Attribute('name', str, False),
    origin_element,
    xmlr.Element('geometry', 'geometric')
])


class Texture(xmlr.Object):
    def __init__(self, filename=None):
        self.filename = filename


xmlr.reflect(Texture, tag='texture', params=[
    xmlr.Attribute('filename', str)
])


class Material(xmlr.Object):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    def check_valid(self):
        if self.color is None and self.texture is None:
            xmlr.on_error("Material has neither a color nor texture.")


xmlr.reflect(Material, tag='material', params=[
    name_attribute,
    xmlr.Element('color', Color, False),
    xmlr.Element('texture', Texture, False)
])


class LinkMaterial(Material):
    def check_valid(self):
        pass


class Visual(xmlr.Object):
    def __init__(self, geometry=None, material=None, origin=None, name=None):
        self.geometry = geometry
        self.material = material
        self.name = name
        self.origin = origin


xmlr.reflect(Visual, tag='visual', params=[
    xmlr.Attribute('name', str, False),
    origin_element,
    xmlr.Element('geometry', 'geometric'),
    xmlr.Element('material', LinkMaterial, False)
])


class Inertia(xmlr.Object):
    KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz

    def to_matrix(self):
        return [
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz]]


xmlr.reflect(Inertia, tag='inertia',
             params=[xmlr.Attribute(key, float) for key in Inertia.KEYS])


class Inertial(xmlr.Object):
    def __init__(self, mass=0.0, inertia=None, origin=None):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin


xmlr.reflect(Inertial, tag='inertial', params=[
    origin_element,
    xmlr.Element('mass', 'element_value'),
    xmlr.Element('inertia', Inertia, False)
])


# FIXME: we are missing the reference position here.
class JointCalibration(xmlr.Object):
    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling


xmlr.reflect(JointCalibration, tag='calibration', params=[
    xmlr.Attribute('rising', float, False, 0),
    xmlr.Attribute('falling', float, False, 0)
])


class JointLimit(xmlr.Object):
    def __init__(self, effort=None, velocity=None, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper


xmlr.reflect(JointLimit, tag='limit', params=[
    xmlr.Attribute('effort', float),
    xmlr.Attribute('lower', float, False, 0),
    xmlr.Attribute('upper', float, False, 0),
    xmlr.Attribute('velocity', float)
])

# FIXME: we are missing __str__ here.


class JointMimic(xmlr.Object):
    def __init__(self, joint_name=None, multiplier=None, offset=None):
        self.joint = joint_name
        self.multiplier = multiplier
        self.offset = offset


xmlr.reflect(JointMimic, tag='mimic', params=[
    xmlr.Attribute('joint', str),
    xmlr.Attribute('multiplier', float, False),
    xmlr.Attribute('offset', float, False)
])


class SafetyController(xmlr.Object):
    def __init__(self, velocity=None, position=None, lower=None, upper=None):
        self.k_velocity = velocity
        self.k_position = position
        self.soft_lower_limit = lower
        self.soft_upper_limit = upper


xmlr.reflect(SafetyController, tag='safety_controller', params=[
    xmlr.Attribute('k_velocity', float),
    xmlr.Attribute('k_position', float, False, 0),
    xmlr.Attribute('soft_lower_limit', float, False, 0),
    xmlr.Attribute('soft_upper_limit', float, False, 0)
])


class Joint(xmlr.Object):
    TYPES = ['unknown', 'revolute', 'continuous', 'prismatic',
             'floating', 'planar', 'fixed']

    def __init__(self, name=None, parent=None, child=None, joint_type=None,
                 axis=None, origin=None,
                 limit=None, dynamics=None, safety_controller=None,
                 calibration=None, mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = joint_type
        self.axis = axis
        self.origin = origin
        self.limit = limit
        self.dynamics = dynamics
        self.safety_controller = safety_controller
        self.calibration = calibration
        self.mimic = mimic

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def joint_type(self): return self.type

    @joint_type.setter
    def joint_type(self, value): self.type = value

xmlr.reflect(Joint, tag='joint', params=[
    name_attribute,
    xmlr.Attribute('type', str),
    origin_element,
    xmlr.Element('axis', 'element_xyz', False),
    xmlr.Element('parent', 'element_link'),
    xmlr.Element('child', 'element_link'),
    xmlr.Element('limit', JointLimit, False),
    xmlr.Element('dynamics', JointDynamics, False),
    xmlr.Element('safety_controller', SafetyController, False),
    xmlr.Element('calibration', JointCalibration, False),
    xmlr.Element('mimic', JointMimic, False),
])


class Link(xmlr.Object):
    def __init__(self, name=None, visual=None, inertial=None, collision=None,
                 origin=None):
        self.aggregate_init()
        self.name = name
        self.visuals = []
        if visual:
            self.visual = visual
        self.inertial = inertial
        self.collisions = []
        if collision:
            self.collision = collision
        self.origin = origin

    def __get_visual(self):
        """Return the first visual or None."""
        if self.visuals:
            return self.visuals[0]

    def __set_visual(self, visual):
        """Set the first visual."""
        if self.visuals:
            self.visuals[0] = visual
        else:
            self.visuals.append(visual)
        if visual:
            self.add_aggregate('visual', visual)

    def __get_collision(self):
        """Return the first collision or None."""
        if self.collisions:
            return self.collisions[0]

    def __set_collision(self, collision):
        """Set the first collision."""
        if self.collisions:
            self.collisions[0] = collision
        else:
            self.collisions.append(collision)
        if collision:
            self.add_aggregate('collision', collision)

    # Properties for backwards compatibility
    visual = property(__get_visual, __set_visual)
    collision = property(__get_collision, __set_collision)


xmlr.reflect(Link, tag='link', params=[
    name_attribute,
    origin_element,
    xmlr.AggregateElement('visual', Visual),
    xmlr.AggregateElement('collision', Collision),
    xmlr.Element('inertial', Inertial, False),
])


class PR2Transmission(xmlr.Object):
    def __init__(self, name=None, joint=None, actuator=None, type=None,
                 mechanicalReduction=1):
        self.name = name
        self.type = type
        self.joint = joint
        self.actuator = actuator
        self.mechanicalReduction = mechanicalReduction


xmlr.reflect(PR2Transmission, tag='pr2_transmission', params=[
    name_attribute,
    xmlr.Attribute('type', str),
    xmlr.Element('joint', 'element_name'),
    xmlr.Element('actuator', 'element_name'),
    xmlr.Element('mechanicalReduction', float)
])


class Actuator(xmlr.Object):
    def __init__(self, name=None, mechanicalReduction=1):
        self.name = name
        self.mechanicalReduction = None


xmlr.reflect(Actuator, tag='actuator', params=[
    name_attribute,
    xmlr.Element('mechanicalReduction', float, required=False)
])


class TransmissionJoint(xmlr.Object):
    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.hardwareInterfaces = []

    def check_valid(self):
        assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"


xmlr.reflect(TransmissionJoint, tag='joint', params=[
    name_attribute,
    xmlr.AggregateElement('hardwareInterface', str),
])


class Transmission(xmlr.Object):
    """ New format: http://wiki.ros.org/urdf/XML/Transmission """

    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.joints = []
        self.actuators = []

    def check_valid(self):
        assert len(self.joints) > 0, "no joint defined"
        assert len(self.actuators) > 0, "no actuator defined"


xmlr.reflect(Transmission, tag='new_transmission', params=[
    name_attribute,
    xmlr.Element('type', str),
    xmlr.AggregateElement('joint', TransmissionJoint),
    xmlr.AggregateElement('actuator', Actuator)
])

xmlr.add_type('transmission',
              xmlr.DuckTypedFactory('transmission',
                                    [Transmission, PR2Transmission]))


class SensorNoise(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """
    TYPES = ['unknown', 'none', 'gaussian', 'gaussian_quantized']

    def __init__(self, noise_type=None, mean=None, stddev=None,
                 bias_mean=None, bias_stddev=None,
                 dynamic_bias_stddev=None, dynamic_bias_correlation_time=None, precision=None):
        self.type = noise_type
        self.mean = mean
        self.stddev = stddev
        self.bias_mean = bias_mean
        self.bias_stddev = bias_stddev
        self.dynamic_bias_stddev = dynamic_bias_stddev
        self.dynamic_bias_correlation_time = dynamic_bias_correlation_time
        self.precision = precision

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid noise type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def noise_type(self): return self.type

    @noise_type.setter
    def noise_type(self, value): self.type = value

xmlr.reflect(SensorNoise, tag='noise', params=[
    xmlr.Attribute('type', str),
    xmlr.Attribute('mean', float, False, 0),
    xmlr.Attribute('stddev', float, False, 0),
    xmlr.Attribute('bias_mean', float, False, 0),
    xmlr.Attribute('bias_stddev', float, False, 0),
    xmlr.Attribute('dynamic_bias_stddev', float, False, 0),
    xmlr.Attribute('dynamic_bias_correlation_time', float, False, 0),
    xmlr.Attribute('precision', float, False, 0),
])


class CameraImage(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, width=None, height=None, image_format=None,
                 hfov=None, near=None, far=None):
        self.width = width
        self.height = height
        self.format = image_format
        self.hfov = hfov
        self.near = near
        self.far = far

xmlr.reflect(CameraImage, tag='image', params=[
    xmlr.Attribute('width', float), # int not allowed by xmlr
    xmlr.Attribute('height', float), # int not allowed by xmlr
    xmlr.Attribute('format', str),
    xmlr.Attribute('hfov', float),
    xmlr.Attribute('near', float),
    xmlr.Attribute('far', float),
])

class SensorCamera(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, image=None):
        self.image = image

xmlr.reflect(SensorCamera, tag='camera', params=[
    xmlr.Element('image', CameraImage),
])


class SingleRay(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, samples=None, resolution=None, min_angle=None, max_angle=None):
        self.samples = samples
        self.resolution = resolution
        self.min_angle = min_angle
        self.max_angle = max_angle

xmlr.reflect(SingleRay, tag='horizontal', params=[
    xmlr.Attribute('samples', float, False, 1), # int not allowed by xmlr
    xmlr.Attribute('resolution', float, False, 1),
    xmlr.Attribute('min_angle', float, False, 0),
    xmlr.Attribute('max_angle', float, False, 0),
])
xmlr.reflect(SingleRay, tag='vertical', params=[
    xmlr.Attribute('samples', float, False, 1), # int not allowed by xmlr
    xmlr.Attribute('resolution', float, False, 1),
    xmlr.Attribute('min_angle', float, False, 0),
    xmlr.Attribute('max_angle', float, False, 0),
])

class RayRange(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, mini=None, maxi=None, resolution=None):
        self.min = mini
        self.max = maxi
        self.resolution = resolution

xmlr.reflect(RayRange, tag='range', params=[
    xmlr.Element('min', float, False, 0),
    xmlr.Element('max', float, False),
    xmlr.Element('resolution', float, False),
])

class SensorRay(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, horizontal=None, vertical=None, rayrange=None):
        self.horizontal = horizontal
        self.vertical = vertical
        self.range = rayrange

xmlr.reflect(SensorRay, tag='ray', params=[
    xmlr.Element('horizontal', SingleRay, False),
    xmlr.Element('vertical', SingleRay, False),
    xmlr.Element('range', RayRange, False),
])


class IMUgyro(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, noise=None):
        self.noise = noise

xmlr.reflect(IMUgyro, tag='gyro', params=[
    xmlr.Element('noise', SensorNoise, False),
])

class IMUacceleration(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, noise=None):
        self.noise = noise

xmlr.reflect(IMUacceleration, tag='acceleration', params=[
    xmlr.Element('noise', SensorNoise, False),
])

class SensorIMU(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, gyro=None, acceleration=None):
        self.gyro = gyro
        self.acceleration = acceleration

xmlr.reflect(SensorIMU, tag='imu', params=[
    xmlr.Element('gyro', IMUgyro, False),
    xmlr.Element('acceleration', IMUacceleration, False),
])


class SensorMagnetometer(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, noise=None):
        self.noise = noise

xmlr.reflect(SensorMagnetometer, tag='magnetometer', params=[
    xmlr.Element('noise', SensorNoise, False),
])


class GPSpostion(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, noise=None):
        self.noise = noise

xmlr.reflect(GPSpostion, tag='position_sensing', params=[
    xmlr.Element('noise', SensorNoise, False),
])

class GPSvelocity(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, noise=None):
        self.noise = noise

xmlr.reflect(GPSvelocity, tag='velocity_sensing', params=[
    xmlr.Element('noise', SensorNoise, False),
])

class SensorGPS(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, position_sensing=None, velocity_sensing=None):
        self.position_sensing = position_sensing
        self.velocity_sensing = velocity_sensing

xmlr.reflect(SensorGPS, tag='imu', params=[
    xmlr.Element('position_sensing', GPSpostion, False),
    xmlr.Element('velocity_sensing', GPSvelocity, False),
])


class SensorForceTorque(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """
    TYPES = ['parent_to_child', 'child_to_parent']

    def __init__(self, frame=None, measure_direction=None):
        self.frame = frame
        self.measure_direction = measure_direction

    def check_valid(self):
        assert self.measure_direction in self.TYPES, "Invalid measure_direction: {}".format(self.measure_direction)  # noqa

xmlr.reflect(SensorForceTorque, tag='froce_torque', params=[
    xmlr.Element('frame', str, False),
    xmlr.Element('measure_direction', str, False),
])


class SensorContact(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, collision=None):
        self.collision = collision

xmlr.reflect(SensorContact, tag='contact', params=[
    xmlr.Element('collision', str, False),
])


class SensorSonar(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self, mini=None, maxi=None, radius=None):
        self.min = mini
        self.max = maxi
        self.radius = radius

xmlr.reflect(SensorSonar, tag='sonar', params=[
    xmlr.Element('min', float, False),
    xmlr.Element('max', float, False),
    xmlr.Element('radius', float, False),
])


class SensorRFIDtag(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self):
        pass

xmlr.reflect(SensorRFIDtag, tag='rfidtag', params=[])


class SensorRFID(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """

    def __init__(self):
        pass

xmlr.reflect(SensorRFID, tag='rfid', params=[])


class Sensor(xmlr.Object):
    """ New format: https://wiki.ros.org/urdf/XML/sensor/proposals """
    TYPES = ['unknown', 'camera', 'ray', 'imu', 'magnetometer', 'gps',
             'force_torque', 'contact', 'sonar', 'rfidtag', 'rfid']

    def __init__(self, name=None, sensor_type=None, sensor_id=None, update_rate=None,
                 parent=None, origin=None,
                 camera=None, ray=None, imu=None, magnetometer=None, gps=None,
                 force_torque=None, contact=None, sonar=None, rfidtag=None, rfid=None):
        self.name = name
        self.type = sensor_type
        self.sensor_id = sensor_id
        self.update_rate = update_rate
        self.parent = parent
        self.origin = origin
        self.camera = camera
        self.ray = ray
        self.imu = imu
        self.magnetometer = magnetometer
        self.gps = gps
        self.force_torque = force_torque
        self.contact = contact
        self.sonar = sonar
        self.rfidtag = rfidtag
        self.rfid = rfid

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid sensor type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def sensor_type(self): return self.type

    @sensor_type.setter
    def sensor_type(self, value): self.type = value

xmlr.reflect(Sensor, tag='sensor', params=[
    name_attribute,
    xmlr.Attribute('type', str),
    xmlr.Attribute('id', str, False),
    xmlr.Attribute('update_rate', float, False),
    xmlr.Element('parent', 'element_link', False),
    origin_element,
    xmlr.Element('camera', SensorCamera, False),
    xmlr.Element('ray', SensorRay, False),
    xmlr.Element('imu', SensorIMU, False),
    xmlr.Element('magnetometer', SensorMagnetometer, False),
    xmlr.Element('gps', SensorGPS, False),
    xmlr.Element('force_torque', SensorForceTorque, False),
    xmlr.Element('contact', SensorContact, False),
    xmlr.Element('sonar', SensorSonar, False),
    xmlr.Element('rfidtag', SensorRFIDtag, False),
    xmlr.Element('rfid', SensorRFID, False),
])


class Robot(xmlr.Object):
    SUPPORTED_VERSIONS = ["1.0"]

    def __init__(self, name=None, version="1.0"):
        self.aggregate_init()

        self.name = name
        if version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))

        self.version = version
        self.joints = []
        self.links = []
        self.materials = []
        self.gazebos = []
        self.transmissions = []
        self.sensors = []

        self.joint_map = {}
        self.link_map = {}
        self.sensor_map = {}

        self.parent_map = {}
        self.child_map = {}

    def add_aggregate(self, typeName, elem):
        xmlr.Object.add_aggregate(self, typeName, elem)

        if typeName == 'joint':
            joint = elem
            self.joint_map[joint.name] = joint
            self.parent_map[joint.child] = (joint.name, joint.parent)
            if joint.parent in self.child_map:
                self.child_map[joint.parent].append((joint.name, joint.child))
            else:
                self.child_map[joint.parent] = [(joint.name, joint.child)]
        elif typeName == 'link':
            link = elem
            self.link_map[link.name] = link
        elif typeName == 'sensor':
            sensor = elem
            self.sensor_map[sensor.name] = sensor

    def add_link(self, link):
        self.add_aggregate('link', link)

    def add_joint(self, joint):
        self.add_aggregate('joint', joint)

    def add_sensor(self, sensor):
        self.add_aggregate('sensor', sensor)

    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            (joint, parent) = self.parent_map[link]
            if joints:
                if fixed or self.joint_map[joint].joint_type != 'fixed':
                    chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    def get_root(self):
        root = None
        for link in self.link_map:
            if link not in self.parent_map:
                assert root is None, "Multiple roots detected, invalid URDF."
                root = link
        assert root is not None, "No roots detected, invalid URDF."
        return root

    def post_read_xml(self):
        if self.version is None:
            self.version = "1.0"

        split = self.version.split(".")
        if len(split) != 2:
            raise ValueError("The version attribute should be in the form 'x.y'")

        if split[0] == '' or split[1] == '':
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if self.version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))

    @classmethod
    def from_parameter_server(cls, key='robot_description'):
        """
        Retrieve the robot model on the parameter server
        and parse it to create a URDF robot structure.

        Warning: this requires roscore to be running.
        """
        # Could move this into xml_reflection
        import rospy
        return cls.from_xml_string(rospy.get_param(key))


xmlr.reflect(Robot, tag='robot', params=[
    xmlr.Attribute('name', str),
    xmlr.Attribute('version', str, False),
    xmlr.AggregateElement('link', Link),
    xmlr.AggregateElement('joint', Joint),
    xmlr.AggregateElement('gazebo', xmlr.RawType()),
    xmlr.AggregateElement('transmission', 'transmission'),
    xmlr.AggregateElement('material', Material),
    xmlr.AggregateElement('sensor', Sensor)
])

# Make an alias
URDF = Robot

xmlr.end_namespace()
