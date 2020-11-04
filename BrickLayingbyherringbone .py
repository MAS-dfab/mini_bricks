from compas_rhino import unload_modules
unload_modules('model')

import Rhino.Geometry as rg
from model import Brick
import math


def brick_horizontal_laying(row, gap):
    planes = []
    meshes = []
    x = Brick.REFERENCE_LENGTH / 2 
    y = Brick.REFERENCE_WIDTH + 1 
    
    for i in range(row):
        x_value = x * i 
        y_value = y * i 
        plane_origin = rg.Point3d(x_value, y_value, 0)
        plane = rg.Plane(plane_origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
        planes.append(plane)
        b = Brick(plane)
        meshes.append(b.mesh())
    return planes, meshes

def brick_Vertical_laying(row, gap):
    planes = []
    meshes = []
    x = Brick.REFERENCE_LENGTH / 2 
    y = Brick.REFERENCE_WIDTH + 1 
    strat_vaule_x = Brick.REFERENCE_WIDTH/2 + 1 
    strat_vaule_y = Brick.REFERENCE_WIDTH/2 + Brick.REFERENCE_LENGTH / 2 + 1
    
    for i in range(row):
        x_value = -strat_vaule_x + x * i 
        y_value = strat_vaule_y + y * i 
        plane_origin = rg.Point3d(x_value, y_value, 0)
        plane = rg.Plane(plane_origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
        plane.Rotate(math.radians(90), rg.Vector3d.ZAxis)
        planes.append(plane)
        b = Brick(plane)
        meshes.append(b.mesh())
    return planes, meshes


planes, meshes_h = brick_horizontal_laying(row,0.5)
planes02, meshes_v = brick_Vertical_laying(row,0.5)

pt_00 = rg.Point3d(0,0,0)
pt_01 = rg.Point3d(x,0,0)
pt_02 = rg.Point3d(0,y,0)
pt_03 = rg.Point3d(x,y,0)

line_00 = rg.NurbsCurve.Create(False, 1, [pt_00, pt_01])
line_01 = rg.NurbsCurve.Create(False, 1, [pt_01, pt_02])
params_00 = line_00.DivideByLength(10,True)
params_01 = line_01.DivideByLength(10,True)
divd_pt01 = line_00.PointAt(params_00)
divd_pt02 = line_01.PointAt(params_01)


conectLine = rg.NurbsCurve.Create(False, 1, [divd_pt01, divd_pt02])



line_03 = rg.NurbsCurve.Create(False, 1, [pt_02, pt_03])
