import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
import math as m
import ghpythonlib.components as ghc
import random
from itertools import combinations

import simple_comm as c
import simple_ur_script as ur

import scriptcontext as sc
import ghpythonremote
import Rhino.Geometry as rg
np = sc.sticky['numpy']
rpy = sc.sticky['rpy']
sklearn = sc.sticky['sklearn']
sklearn.cluster = sc.sticky['sklearn.cluster']


class Fabrication():
    ROBOT_IP = "192.168.10.10"

    def __init__(self, fabricate=False, brick_planes=None, robot_ip=ROBOT_IP):
        self.brick_planes = brick_planes
        self.robot_ip = robot_ip
        self.accel = 0.1
        self.vel = 0.1
        self.script = ""
        self.way_planes = []

    def tcp(self):
        self.script += ur.set_tcp_by_angles(
            0.0, 0.0, 74.0, m.radians(0.0), m.radians(180.0), m.radians(0))

    def set_robot_base_plane(self):

        rs.MessageBox(
            "move robot to base plane origin, press OK when there", 0)
        data = c.listen_to_robot(ROBOT_IP)
        pose = data['pose']
        pt_1 = rg.Point3d(pose[0]*1000, pose[1]*1000, pose[2]*1000)
        print(pt_1)
        rs.MessageBox(
            "move robot to base plane positive x direction, press OK when there", 0)
        data = c.listen_to_robot(ROBOT_IP)
        pose = data['pose']
        pt_2 = rg.Point3d(pose[0]*1000, pose[1]*1000, pose[2]*1000)
        print(pt_2)
        rs.MessageBox(
            "move robot to base plane positive y direction, press OK when there", 0)
        data = c.listen_to_robot(robot_ip)
        pose = data['pose']
        pt_3 = rg.Point3d(pose[0]*1000, pose[1]*1000, pose[2]*1000)
        print(pt_3)

        robot_base = rg.Plane(pt_1, pt_2-pt_1, pt_3-pt_1)
        text_file = open("robot_base.txt", "w")
        text_file.write(str(robot_base.Origin)+"," +
                        str(robot_base.XAxis)+","+str(robot_base.YAxis))
        text_file.close()

    # def load_robot_base_plane(self):
        text_file = open("robot_base.txt", "r")
        string = text_file.read()
        values = string.split(",")
        values = [float(value) for value in values]

        base_origin = rg.Point3d(values[0], values[1], values[2])
        base_x_axis = rg.Vector3d(values[3], values[4], values[5])
        base_y_axis = rg.Vector3d(values[6], values[7], values[8])

        base_plane = rg.Plane(base_origin, base_x_axis, base_y_axis)
        return base_plane

    def set_robot_base_plane_from_pts(self):

        pt_0 = rg.Point3d(327, 230, 0)  # base plane origin
        # point on base plane positive x direction
        pt_1 = rg.Point3d(499, 230, 0)
        pt_2 = rg.Point3d(499, 499, 0)  # point on base plane positive xy

        robot_base = rg.Plane(pt_0, pt_1-pt_0, pt_2-pt_0)
        return robot_base

    def rhino_to_robot_space(self, in_plane):
        plane = in_plane.Clone()
        _r_matrix = rg.Transform.PlaneToPlane(
            rg.Plane.WorldXY, self.set_robot_base_plane_from_pts())
        plane.Transform(_r_matrix)
        return plane

    def robot_transformation(self):  # LOAD robot model
        _robot_matrix = rg.Transform.PlaneToPlane(
            self.set_robot_base_plane_from_pts(), rg.Plane.WorldXY)
        return _robot_matrix

    def pickup_brick(self, pick_up_plane):
        """Example of a method's documentation.

        Parameters
        ----------
        point_x : float
        Description of the first param
        point_y : float
        Description of the second param
        """

        safe_distance = 50

        safe_plane = pick_up_plane.Clone()
        safe_plane.Translate(rg.Vector3d(0, 0, safe_distance))

        self.script += ur.move_l(safe_plane, self.accel, self.vel)
        self.way_planes.append(safe_plane)

        self.script += ur.move_l(pick_up_plane, self.accel, self.vel)
        self.way_planes.append(pick_up_plane)

        self.script += ur.set_digital_out(4, True)
        self.script += ur.sleep(1)

        self.script += ur.move_l(safe_plane, self.accel, self.vel)
        self.way_planes.append(safe_plane)

        return None

    def place_brick(self, plane):
        """
        this function gereates the robotic sequience for placing a brick
        Requires a plane wich describes the possition of the brick"""

        safe_distance = 50

        safe_plane = plane.Clone()
        safe_plane.Translate(rg.Vector3d(0, 0, safe_distance))

        self.script += ur.move_l(safe_plane, self.accel, self.vel)
        self.way_planes.append(safe_plane)

        self.script += ur.move_l(plane, self.accel, self.vel)
        self.way_planes.append(plane)

        self.script += ur.set_digital_out(4, False)
        self.script += ur.sleep(1)

        self.script += ur.move_l(safe_plane, self.accel, self.vel)
        self.way_planes.append(safe_plane)

        return None

    def procedure(self, transform=True):

        self.tcp()
        robot_planes = []

        if transform:
            pick_up_plane = rg.Plane(rg.Point3d(
                0, 450, 0), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
            for plane in (self.brick_planes):
                robot_planes.append(self.rhino_to_robot_space(plane))

        else:
            pick_up_plane = self.rhino_to_robot_space(
            rg.Plane(rg.Point3d(0, 450, 0), rg.Vector3d.XAxis, rg.Vector3d.YAxis))
            robot_planes = self.brick_planes

            for plane in (robot_planes):
                self.pickup_brick(pick_up_plane)
                self.place_brick(plane)

    def visualize(self):
        """this funktion visualizes the planes wich are sent to the robot"""

        crv = []

        for plane in self.way_planes:
            #print (plane.Origin)
            pt = plane.Origin
            crv.append(pt)

        curve = rg.NurbsCurve.Create(False, 1, crv)
        print(curve)
        return self.way_planes, curve

    def send(self):
        """
        this funktion sends the script to the robot."""

        self.script = c.concatenate_script(self.script)
        #c.send_script(self.script, self.robot_ip)
        return self.script

"""Define global Area list and dictionary
"""
areas_dict = {}
areas_list = []


class Area():

    def __init__(self, initial_curve, start_param, end_param, layer, area_index, width=100):
        """Area describes the bounding rectangle in which bricks are placed

        Parameters
        ----------
        plane : Rhino Geometry plane
        this plane describes the possition and orientation of the Brick

        """
        self.initial_curve = initial_curve.Duplicate()
        self.start_param = start_param
        self.end_param = end_param
        self.width = width
        self.layer = layer
        self.area_index = area_index
        self.area_key = "L{}A{}".format(layer, area_index)
        self.sub_layer_neighbors = []
        self.prev_neighbor = self.get_previous_neighbor(self.layer, self.area_index)
        self.initial_number_bricks = 0
        self.area_bricks = []
        self.RTree = None

    def centerline(self, start=False, end=False):
        """get centerline of the area

        Returns
        ----------
        Rhino Geometry Curve
        """
        if start is False:
            start = self.start_param
        if end is False:
            end = self.end_param

        splits = self.initial_curve.Split((start, end))
        if len(splits) == 2:
            return splits[0]
        else:
            return splits[1]

    def start_point(self):
        """get point at start of area positioned on the initial curve

        Returns
        ----------
        Rhino Geometry Point3d
        """
        return rg.Curve.PointAt(self.initial_curve, self.start_param)

    def end_point(self):
        """get point at end of area positioned on the initial curve

        Returns
        ----------
        Rhino Geometry Point3d
        """
        return rg.Curve.PointAt(self.initial_curve, self.end_param)

    def extend_bounding_area(self, both_sides=False):
        """Extend the bounding area both sides of the area along the initial curve

        Parameters
        ----------
        offset : float
        this describes the length of the offset of the area in both directions

        Returns
        ----------
        Rhino Geometry Poly Curve
        """
        offset = Brick.REFERENCE_LENGTH
        len_curve = self.initial_curve.GetLength()

        # extend start parameter
        len = self.initial_curve.GetLength(rg.Interval(self.initial_curve.Domain[0], self.start_param))
        len_offset = len - offset

        if len_offset <= 0.0:
            new_start_param = self.initial_curve.Domain[0]
        else:
            _ , new_start_param = self.initial_curve.LengthParameter(len_offset)
        
        new_end_param = self.end_param

        if both_sides is True:
            # extend end parameter
            len = self.initial_curve.GetLength(rg.Interval(self.initial_curve.Domain[0], self.end_param))
            len_offset = len + offset 
            if len_offset >= len_curve:
                new_end_param = self.initial_curve.Domain[1]
            else:
                _ , new_end_param = self.initial_curve.LengthParameter(len_offset)

        
        # create offset from centerline
        curve0 = self.centerline(start=new_start_param, end=new_end_param).Offset(rg.Plane.WorldXY, self.width/2, 0.001, 0)
        curve0 = curve0[0]
        # offset other direction
        curve1 = self.centerline(start=new_start_param, end=new_end_param).Offset(rg.Plane.WorldXY, self.width/2*-1, 0.001, 0)
        curve1 = curve1[0]
        # use start and end points of offset line to connect and create bounding area
        curve2 = rg.LineCurve( curve0.PointAtStart, curve1.PointAtStart )
        curve3 = rg.LineCurve( curve0.PointAtEnd, curve1.PointAtEnd )
        curve_collection = rg.Curve.JoinCurves((curve0, curve1, curve2, curve3))
        
        return curve_collection[0]

    def project_bbox_extend_on_ground(self):
        height_layer = self.layer * Brick.REFERENCE_HEIGHT
        V = rg.Vector3d.ZAxis * height_layer * -1
        copy_curve = self.extend_bounding_area().Duplicate()
        copy_curve.Translate(V)
        return copy_curve.GetBoundingBox(False)

    def sub_layer(self):
        if self.layer > 0:
            return self.layer - 1
        else:
            return None

    def get_previous_neighbor(self, layer, index):
        if index == 0:
            return None
        else:
            return "L{}A{}".format(layer, index-1)

    def tangent_origin(self):
        """returns the tangent at the origin plane of the area:

        Returns
        ----------
        [Rhino Geometry Plane]

        """
        params = rg.Curve.DivideByCount(self.centerline, 2, True)
        center_point = rg.Curve.PointAt(self.centerline, params[1])
        tan = rg.Curve.TangentAt(self.centerline, params[1])
        vec1 = rg.Vector3d.CrossProduct(tan, rg.Vector3d.ZAxis)
        vec2 = rg.Vector3d.CrossProduct(vec1, rg.Vector3d.ZAxis)
        plane = rg.Plane(center_point, vec1, vec2)

        return plane

    def bounding_area(self):
        """Create the bounding area:

        Returns
        ----------
        [Rhino Geometry PolyLine Collection]

        """
        # create offset from centerline
        curve0 = self.centerline().Offset(rg.Plane.WorldXY, self.width/2, 0.001, 0)
        curve0 = curve0[0]
        # offset other direction
        curve1 = self.centerline().Offset(rg.Plane.WorldXY, self.width/2*-1, 0.001, 0)
        curve1 = curve1[0]
        # use start and end points of offset line to connect and create bounding area
        curve2 = rg.LineCurve( curve0.PointAtStart, curve1.PointAtStart )
        curve3 = rg.LineCurve( curve0.PointAtEnd, curve1.PointAtEnd )
        curve_collection = rg.Curve.JoinCurves((curve0, curve1, curve2, curve3))
        
        return curve_collection[0]

    def floating_edges(self, clustered_pts):
        """Create the delauney triangulation and pick up floating edges:

        Returns
        ----------
        [Rhino Geometry Line Collection]

        """
        floating_edges = []

        bl = Brick.REFERENCE_LENGTH
        bw = Brick.REFERENCE_WIDTH
        bd = m.sqrt( (bl/2.0)*(bl/2.0) + bw*bw ) # diagonal edge of brick
        tol = 0.01                               # tolerance of range (if necessary)
        
        # create edges inside the picked up cluster
        clusterd_edges = ghc.DelaunayEdges( clustered_pts )[1]

        for edge in clusterd_edges:
            edge_length = edge.Length
            if ( ( (bl-tol) < edge_length )  or                         #remove longer edges of the bricks and edges longer than bl
                 ( (bw-tol) < edge_length < (bw+tol) )  or              #remove shorter edges of the bricks 
                 ( (bd-tol) < edge_length < (bd+tol) )  or              #remove diagonal edges of the bricks
                 ( ((bl/2.0)-tol) < edge_length < ((bl/2.0)+tol) ) ):   #remove divided longer edges of the bricks
                continue
            else:
                floating_edges.append( edge )
        return floating_edges


    def average_planes(self, lines):
        s_x = 0
        s_y = 0
        s_z = 0
        
        e_x = 0
        e_y = 0
        e_z = 0
        
        l = len(lines)
        if l == 0 :
            pass

        else:
            for line in lines:
                startPt = line.PointAt(0.0)
                endPt = line.PointAt(1.0)
                midPt = line.PointAt(0.5)
                
                startPt_2 = line.PointAt(0.0)
                endPt_2 = line.PointAt(1.0)
                
                s_x += startPt.X
                s_y += startPt.Y
                s_z += startPt.Z
                
                e_x += endPt.X
                e_y += endPt.Y
                e_z += endPt.Z
                
            
            
            sx = s_x/l
            sy = s_y/l
            sz = s_z/l
            
            ex = e_x/l
            ey = e_y/l
            ez = e_z/l
            
            a = rg.Point3d(sx, sy, sz)
            b = rg.Point3d(ex, ey, ez)
            
            
            
            aV = rg.Vector3d(a)
            bV = rg.Vector3d(b)
            c = rg.Vector3d(bV-aV)
            
            ave_l = rg.Line(a, b)
            midP = ave_l.PointAt(0.5)
            
            ave_l2 = rg.Line(a, b)
            T = rg.Transform.Rotation(rg.Vector3d.XAxis, rg.Vector3d.YAxis, midPt)
            ave_l2.Transform(T)
            aV2 = rg.Vector3d( rg.Line.PointAt(ave_l2, 0.5) )
            bV2 = rg.Vector3d( rg.Line.PointAt(ave_l2, 1.0) )
            c2 = rg.Vector3d(bV2 - aV2)
            
            midP2 = rg.Point3d(midP.X, midP.Y, midP.Z+1)
            
            sa = rg.Vector3d(midP)
            ea = rg.Vector3d(midP2)
            axis = ea - sa
            
            ave_plane = rg.Plane(midP, c, c2)
            return ave_plane

    def get_bricks_vertices(self):
        vertices = []
        for brick in self.area_bricks:
            vertices.extend(brick.pts_before_clustering())
        return vertices

    def find_points_in_extend(self, point_list):
        inside_pts = []
        for p in point_list:
            if self.extend_bounding_area().Contains(p) == rg.PointContainment.Inside:
                inside_pts.append(p)
        return inside_pts

    def num_brick(self, area, density):
        """this function calculates the number of bricks per area

        Parameters
        ----------
        area : Rhino Geometry polycurve
        this area describes the curve of bbox for each area

        Returns
        ----------
        num_bricks : int
        this describes the number of bricks per area

        """
        self.area = area
        self.density = density

        #calculate the area of a brick
        bl = Brick.REFERENCE_LENGTH
        bw = Brick.REFERENCE_WIDTH
        ba = bl * bw
        
        #calculate the are of a bounding_area
        b_mass_pro = rg.AreaMassProperties.Compute( self.area )
        if b_mass_pro == None:
            num_brick = 0
            pass
        else:
            b_area = b_mass_pro.Area
            num_brick = int ( (b_area*self.density) / ba )
        self.initial_number_bricks = num_brick
        return num_brick

    def populate_points(self, area, num_brick):
        """This function create the list of points per area 
           This should be run ONLY for ground plane

        Parameters
        ----------
        area      : Rhino Geometry polycurve
                     this area describes the curve of bbox for each area
        num_brick : Rhino Geometry polycurve
                     this area describes the curve of bbox for each area

        Returns
        ----------
        pts       : RhinoGeometry Point3d
                     this describes the points per area 

        """
        self.area = area
        self.num_brick = num_brick
        pts = ghc.Populate2D(region=self.area, count=self.num_brick, seed=random.randint(0,100))
        if pts == None:
            pass
        else:
            return pts

    def create_R_tree_from_bricks(self, brickObjs):
        brick_origins = [ b.origin().Origin for b in brickObjs ]
        tree = rg.RTree.CreateFromPointArray(brick_origins)
        self.RTree = tree
        

    def check_intersecting_bricks(self):
        """Checks if bricks in area are intersecting with each other or with bricks from previous area

        Returns
        ----------
        Boolean     : Boolean 
                     True if bricks intersect
        """
        global areas_dict

        # initiate count number of intersecting bricks for Area
        n_intersecting_bricks = 0

        # if no bricks in area_bricks then return
        if len(self.area_bricks) == 0:
            return n_intersecting_bricks, [] #, "there are no bricks in this area"

        # initiate number of intersections to zero for each brick
        for brick in self.area_bricks:
            brick.n_intersections = 0

        intersections = []
        # check intersections within the area
        for pair in combinations(self.area_bricks, 2):
            if pair[0].brick_intersect_brick(pair[1]) is True:
                n_intersecting_bricks += 1
                intersections.append((pair[0], pair[1]))
                # update brick property with intersection
                pair[0].n_intersections += 1
                pair[1].n_intersections += 1

        # check intersections with previous Area
        if self.prev_neighbor:
            bricks_neigh = areas_dict[self.prev_neighbor].area_bricks
            # check if brick is inside extend area
            for b1 in self.area_bricks:
                for b2 in bricks_neigh:
                    if b1.brick_intersect_brick(b2) == True:
                        n_intersecting_bricks += 1
                        intersections.append((b1, b2))
                        # update brick property with intersection
                        b1.n_intersections += 1
                        b2.n_intersections += 1


        return n_intersecting_bricks, intersections

    def cluster_sub_layer_points(self):
        """this function uses sklearn.cluster.KMeans to cluster the vertices of sub-layer areas
        """
        if len(self.sub_layer_neighbors) == 0:
            return "could not find any sub-layer points. Current layer is {}".format(self.layer), [], []
        
        global areas_dict

        pts_to_cluster_all = []
        for area in self.sub_layer_neighbors:
            pts_to_cluster_all.extend(areas_dict[area].get_bricks_vertices())
        
        pts_to_cluster_inside = self.find_points_in_extend(pts_to_cluster_all)
        input_data = [ [p.X, p.Y, p.Z] for p in pts_to_cluster_inside ]
        input_data = np.array(input_data, dtype=np.float16)

        kmeans = sklearn.cluster.KMeans(n_clusters=self.initial_number_bricks, n_init=3, max_iter=50).fit(input_data)

        clusters = []
        points = []
        labels = kmeans.labels_.tolist()
        for i in range(self.initial_number_bricks):
            clusters.append([])

        for i, point in zip(kmeans.labels_, pts_to_cluster_inside):
            #point = rg.Point3d(point[0], point[1], point[2])
            clusters[i].append(point)
            points.append(point)
        
        return clusters, points, labels

    
    def random_brick_planes(self, points):
        planes = []
        self.points = points
        if self.points == None:
            pass
        else:
            for point in self.points:
                plane = rg.Plane(point, rg.Vector3d.ZAxis)
                start_vec = rg.Vector3d.XAxis
                end_vec = rg.Vector3d( random.random(), random.random(), 0)
                T = rg.Transform.Rotation( start_vec, end_vec, plane.Origin )
                plane.Transform(T)
                planes.append(plane)
        return planes

    def find_floating_bricks(self):
        """This function will check for every brick inside the area if its floating
        """
        global areas_dict
        floating_bricks = []
        # first find the intersecting bricks from the bottom layer to check with
        all_sub_layer_bricks = []
        for area_key in self.sub_layer_neighbors:
            all_sub_layer_bricks.extend(areas_dict[area_key].area_bricks)
        
        # check bounding box intersections to find the supporting bricks
        for brick_top in self.area_bricks:
            supporting_bricks = []
            for brick_bottom in all_sub_layer_bricks:
                if brick_top.brick_intersect_brick(brick_bottom) is True:
                    supporting_bricks.append(brick_bottom)
            
            # when we know the supporting bricks from the bottom layer for each brick of the top layer we can check if its floating
            if brick_top.check_floating_brick(supporting_bricks) is True:
                floating_bricks.append(brick_top)
        
        return floating_bricks


class Brick(object):
    REFERENCE_LENGTH = 25.0
    REFERENCE_WIDTH = 12.0
    REFERENCE_HEIGHT = 8.0

    def __init__(self, plane, length=REFERENCE_LENGTH, width=REFERENCE_WIDTH, height=REFERENCE_HEIGHT):
        """Brick containes picking plane, placing plane and geometry

        Parameters
        ----------
        plane : Rhino Geometry plane
        this plane describes the possition and orientation of the Brick

        """
        self.plane = plane
        self.length = length
        self.width = width
        self.height = height
        self.n_intersections = 0
        self.floating = False


    def dimensions(self):
        """returns the dimenesnions of the brick:

        Returns
        ----------
        [length : float, width : float, height: float]

        """

        return self.length, self.width, self.height

    def pts(self):
        """returns 8 points describing the brick:

        Returns
        ----------
        [pt0 : bottom point at origin,
        pt1 : bottom point at possitive Y,
        pt2 : bottom point at possitive X,
        pt3 : bottom point at possitive X and poossitive Y,
        pt4 : top point at origin,
        pt1 : top point at possitive Y,
        pt2 : top point at possitive X,
        pt3 : top point at possitive X and poossitive Y]

        """

        pt_0 = rg.Point3d(0, 0, 0)
        pt_1 = rg.Point3d(self.length, 0, 0)
        pt_2 = rg.Point3d(self.length, self.width, 0)
        pt_3 = rg.Point3d(0, self.width, 0)

        pt_4 = rg.Point3d(0, 0, self.height)
        pt_5 = rg.Point3d(self.length, 0, self.height)
        pt_6 = rg.Point3d(self.length, self.width, self.height)
        pt_7 = rg.Point3d(0, self.width, self.height)

        b_pts = [pt_0, pt_1, pt_2, pt_3, pt_4, pt_5, pt_6, pt_7]

        return b_pts



    """
    Ko edited from here
    """
    
    def mid_pts(self):
        """returns 4 points at the middle of longer edges:

        Returns
        ----------
        [mid_pt0 : bottom point 1
         mid_pt1 : bottom point 2
         mid_pt2 : top point 1,
         mid_pt3 : top point 2]

        """
        mid_pt_0 = rg.Point3d(self.length /2.0, 0.0, 0.0)
        mid_pt_1 = rg.Point3d(self.length /2.0, self.width, 0.0)
        mid_pt_2 = rg.Point3d(self.length /2.0, 0.0, self.height)
        mid_pt_3 = rg.Point3d(self.length /2.0, self.width, self.height)

        mid_pts = [mid_pt_0, mid_pt_1, mid_pt_2, mid_pt_3]

        return mid_pts #get middle points of longer edges


    def pts_before_clustering(self):
        """returns list of corner points and mid points of longer edge on top surface

        Returns
        ----------
        tran_brick_pts : list of points

        """
        tran_brick_pts = []
        pts = self.pts()[4:]
        mid_pts = self.mid_pts()[2:]

        for pt in  pts:
            transformation_pt = pt.Clone() #duplicate function for point (for plane, duplicate)
            transformation_pt.Transform(self.transformation())
            tran_brick_pts.append(transformation_pt)   

        for mid_pt in  mid_pts:
            transformation_mid_pt = mid_pt.Clone() #duplicate function for point (for plane, duplicate)
            transformation_mid_pt.Transform(self.transformation())
            tran_brick_pts.append(transformation_mid_pt)    

        return tran_brick_pts

    """
    Ko edited to here
    """


    def origin(self):
        """returns the origin plane in the centre of the base of the brick:

        Returns
        ----------
        [Rhino Geometry Plane]

        """
        vec = (self.pts()[2]-self.pts()[0])/2
        origin = self.pts()[0] + vec
        plane = rg.Plane(origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
        return plane

    def transformation(self):
        """Transoformation matrix fot transformating the brick to the brick possiotn:

        Returns
        ----------
        [Rhino Geometry Transformation]

        """
        return rg.Transform.PlaneToPlane(self.origin(), self.plane)

    def base_plane(self):
        """Base plane of the transformed brick:

        Returns
        ----------
        [Rhino Geometry Plane]

        """

        p_plane = self.origin()
        p_plane.Transform(self.transformation())
        return p_plane

    def picking_plane(self):
        """Robot's picking plane on the transformed brick:

        Returns
        ----------
        [Rhino Geometry Plane]

        """
        p_plane = self.origin()
        p_pt = p_plane.Origin
        p_plane = rg.Plane(p_pt+rg.Vector3d(0, 0, self.height),
                           p_plane.XAxis, p_plane.YAxis)

        p_plane.Transform(self.transformation())
        return p_plane

    def surface(self):
        """NURB surfaces depicting the brick:

        Returns
        ----------
        [srf0 : base surface,
        srf1 : long edge,
        srf2 : top surface
        srf3 : long edge,
        srf4 : short edge
        srf5 : short edge]

        """
        tran_brick_pts = []
        for pt in self.pts():
            transformation_pt = pt.Clone()
            transformation_pt.Transform(self.transformation())
            tran_brick_pts.append(transformation_pt)

        pt_0, pt_1, pt_2, pt_3, pt_4, pt_5, pt_6, pt_7 = tran_brick_pts

        srf_0 = rg.NurbsSurface.CreateFromPoints(
            [pt_0, pt_1, pt_3, pt_2], 2, 2, 1, 1)
        srf_1 = rg.NurbsSurface.CreateFromPoints(
            [pt_0, pt_1, pt_4, pt_5], 2, 2, 1, 1)
        srf_2 = rg.NurbsSurface.CreateFromPoints(
            [pt_4, pt_5, pt_7, pt_6], 2, 2, 1, 1)
        srf_3 = rg.NurbsSurface.CreateFromPoints(
            [pt_6, pt_7, pt_2, pt_3], 2, 2, 1, 1)
        srf_4 = rg.NurbsSurface.CreateFromPoints(
            [pt_0, pt_3, pt_4, pt_7], 2, 2, 1, 1)
        srf_5 = rg.NurbsSurface.CreateFromPoints(
            [pt_1, pt_2, pt_5, pt_6], 2, 2, 1, 1)

        return (srf_0, srf_1, srf_2, srf_3, srf_4, srf_5)

    def mesh(self):
        """Mesh  depicting the brick:

        Returns
        ----------
        mesh_brick : Mesh
        """

        mesh_brick = rg.Mesh.CreateFromBox(self.pts(), 1, 1, 1)
        mesh_brick.Transform(self.transformation())

        return mesh_brick

    def get_bounding_box(self):
        """get bounding box in XY plane projected on ground surface
        """
        return self.mesh().GetBoundingBox(False)

    def move_brick_pts(self, pts):

        tran_brick_pts = []

        for pt in  pts:
            transformation_pt = pt.Clone() #duplicate function for point (for plane, duplicate)
            transformation_pt.Transform(self.transformation())
            tran_brick_pts.append(transformation_pt)      

        return tran_brick_pts

    def brick_intersect_brick(self, other_brick):
        corner_pts_a = self.move_brick_pts(self.pts()[:4])
        corner_pts_b = other_brick.move_brick_pts(other_brick.pts()[:4])
        bbox_a = rg.BoundingBox(corner_pts_a)
        bbox_b = rg.BoundingBox(corner_pts_b)
        if (bbox_a.Min.X <= bbox_b.Max.X) and (bbox_a.Max.X >= bbox_b.Min.X) and (bbox_a.Min.Y <= bbox_b.Max.Y) and (bbox_a.Max.Y >= bbox_b.Min.Y): #and (bbox_a.Min.Z < bbox_b.Max.Z) and (bbox_a.Max.Z > bbox_b.Min.Z):
            poly_a = rg.PolyCurve()
            poly_a.Append(rg.Line(corner_pts_a[0], corner_pts_a[1]))
            poly_a.Append(rg.Line(corner_pts_a[1], corner_pts_a[2]))
            poly_a.Append(rg.Line(corner_pts_a[2], corner_pts_a[3]))
            poly_a.Append(rg.Line(corner_pts_a[3], corner_pts_a[0]))
            for p in corner_pts_b:
                containment = poly_a.Contains(p)
                if containment == rg.PointContainment.Inside or containment == rg.PointContainment.Coincident:
                    return True
            return False
        else:
            return False

    def check_floating_brick(self, sub_layer_bricks):
        # create flat breps from bricks
        # create brep from bottom layer of brick for top brick (self)
        corner_pts_a = self.move_brick_pts(self.pts()[:4])
        brep_top = rg.Brep.CreateFromCornerPoints(corner_pts_a[0], corner_pts_a[1], corner_pts_a[2], corner_pts_a[3], 0.05)
        # create breps of top layer of sub-layer bricks
        sub_layer_breps = []
        for brick in sub_layer_bricks:
            corner_pts_b = brick.move_brick_pts(self.pts()[4:])
            sub_layer_breps.append(rg.Brep.CreateFromCornerPoints(corner_pts_b[0], corner_pts_b[1], corner_pts_b[2], corner_pts_b[3], 0.05))

        intersection_vertices = []
        for b in sub_layer_breps:
            intersection_area = rs.IntersectBreps(brep_top, b, tolerance = None)
            vertices = rs.PolylineVertices(intersection_area[0])
            intersection_vertices.extend(vertices)

        convex_hull = ghc.ConvexHull(intersection_vertices)[0]
        if convex_hull.Contains(self.base_plane().Origin) == rg.PointContainment.Inside:
            self.floating = True
            return True
        else:
            return False


class Wall():
    def __init__(self, x_cnt, z_cnt):
        """Wall generates and contains the bricks

        Parameters
        ----------
        x_cnt : int
        this number corresponds to the length of the wall (amount of bricks)

        x_cnt : int
        this number corresponds to the hight of the wall
        (amount of brick layers)

        """

        self.x_cnt = x_cnt
        self.z_cnt = z_cnt


        self.b_length = Brick.REFERENCE_LENGTH
        self.b_width = Brick.REFERENCE_WIDTH
        self.b_height = Brick.REFERENCE_HEIGHT

    def brick_possitions(self):
        """Genrates a Rhino Geometry plane for each brick

        This is where the design of the wall is created.

        Returns
        ----------
        brick_planes : [Rhino Geometry Plane, Rhino Geometry Plane, ...]

        """

        brick_planes = []
        for i in range(self.z_cnt):  # layer count
            for j in range(self.x_cnt):  # layer length

                if i % 2 == 0:
                    x_pos = j * (self.b_length+1)
                    rotation = m.radians(10)
                else:
                    x_pos = j * (self.b_length+1) + (self.b_length/2)-1
                    rotation = m.radians(-10)

                z_pos = i * (self.b_height)

                origin = rg.Point3d(x_pos, 0, z_pos)
                plane = rg.Plane(origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
                plane.Rotate(rotation, rg.Vector3d.ZAxis)
                brick_planes.append(plane)

        return brick_planes

    def geometric_model(self):
        """Generates a 3D model of the wall

        Returns
        ----------
        geo : [Rhino Geometry Surface, Rhino Geometry Surface, ...]

        """

        planes = []
        geo = []

        for plane in self.brick_possitions():
            myBrick = Brick(plane)
            planes.append(myBrick.base_plane())
            # geo.extend(myBrick.surface())

            geo.append(myBrick.mesh())

        visualizeFabrication = Fabrication(
            brick_planes=self.brick_possitions())
        visualizeFabrication.procedure(transform=False)
        robot_matrix = visualizeFabrication.robot_transformation()

        print(robot_matrix)

        return geo, visualizeFabrication.visualize(), robot_matrix

    def fabrication_model(self):
        """Generates all the data necessary for the robotic fabrication
        process and sends the comands to the robot

        Returns script
        ----------
        script : "UR script"
        """

        myFabrication = Fabrication(brick_planes=self.brick_possitions())

        myFabrication.procedure()
        script = myFabrication.send()

        return script, myFabrication.visualize()
