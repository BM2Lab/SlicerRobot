import numpy as np
import vtk
import json
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import splprep, splev

class MathHelper:
    def __init__(self):
        pass
    @staticmethod
    def convert2SlicerTransform(vtk_matrix):
        '''
        The slicer uses RAS coordinate system, need to flip x and y
        '''
        trans = np.zeros(3)  
        for i in range(3):
            trans[i] = vtk_matrix.GetElement(i, 3)
        rot = np.eye(3)
        for i in range(3):
            for j in range(3):
                rot[i,j] = vtk_matrix.GetElement(i, j)
        # some visual transform has 1000 scale, need to store the scale 
        scale = np.linalg.det(rot)**(1/3)
        trans[0] = -trans[0]
        trans[1] = -trans[1]
        quat = R.from_matrix(rot).as_quat() # [qx, qy, qz, w]
        quat[0] = -quat[0]
        quat[1] = -quat[1]
        rot = R.from_quat(quat).as_matrix()
        rot = rot*scale
        vtk_matrix.SetElement(0, 3, trans[0])
        vtk_matrix.SetElement(1, 3, trans[1])
        vtk_matrix.SetElement(2, 3, trans[2])
        for i in range(3):
            for j in range(3):
                vtk_matrix.SetElement(i, j, rot[i,j])

        return vtk_matrix
    
    @staticmethod
    def transformWaypoints(waypoints, vtk_matrix):
        '''
        Transform waypoints given vtk matrix
        '''
        np_matrix = MathHelper.vtkMatrixToNpMatrix(vtk_matrix)
        converted_waypoints = waypoints @ np_matrix[:3,:3].T + np_matrix[:3,3]
        return converted_waypoints
    
    @staticmethod
    def npMatrixToVtkMatrix(np_matrix):
        '''
        Convert numpy matrix to vtk matrix
        '''
        vtk_matrix = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtk_matrix.SetElement(i, j, np_matrix[i,j])
        return vtk_matrix
    
    @staticmethod
    def vtkMatrixToNpMatrix(vtk_matrix):
        '''
        Convert vtk matrix to numpy matrix
        '''
        np_matrix = np.zeros((4,4))
        for i in range(4):
            for j in range(4):
                np_matrix[i,j] = vtk_matrix.GetElement(i,j)
        return np_matrix
    
    @staticmethod
    def string2Array(array_string):
        """Convert array string back to numpy array using json.loads"""
        try:
            # Parse the string as JSON
            array_list = json.loads(array_string)
            # Convert to numpy array
            return np.array(array_list)
        except json.JSONDecodeError as e:
            # print(f"Error parsing JSON string: {e}") usually means the string is empty
            return None
