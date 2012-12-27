"""
About Me:
website: http://www.ibaiyang.org
Author: Valley.He
Email: ubaiyang@gmail.com
"""

import numpy as np
from math import *
from PyQt4 import QtCore, QtGui

#Define some global variables
VA_MODELVIEW, VA_PROJECTION = 0, 1
VA_LINES, VA_POINTS, VA_LINE_LOOP = 2, 3, 4

"""
Opengl includes two stack to store current matrix state.
proj_mat_stack is used to store projection matrix.
view_mat_stack is used to store modelview matrix.
"""
proj_mat_stack = []
view_mat_stack = []

"""
current matrix mode(VA_MODELVIEW, VA_PROJECTION).
current projection matrix value g_matrix_proj.
current modelview matrix value g_matrix_view.
"""
g_matrix_mode = None
g_matrix_proj = np.asmatrix( np.identity(4, dtype=np.float) )
g_matrix_view = np.asmatrix( np.identity(4, dtype=np.float) )

class VaBuffer():
    """
    Define valley buffer to store each object. Before drawing objects, We should
    know how to draw, for example, points, lines or line loop. And we also should
    store modelview and projection matrix to render it to screen.

    Valley buffer is created when calling vaBegin function.
    """
    def __init__(self, cmd, model, proj):
        self.cmd = cmd
        self.pts_buffer = []
        self.modelview = model
        self.projection = proj

#store valley buffer for each object to objects_buffer list
objects_buffer = []
g_va_buffer = None

def vaBegin(cmd):
    global g_va_buffer
    assert g_va_buffer == None
    g_va_buffer = VaBuffer(cmd, g_matrix_view.copy(), g_matrix_proj.copy())

def vaVertex(x, y, z):
    assert g_va_buffer != None
    p = [x, y, z, 1]
    g_va_buffer.pts_buffer.append(p)

def vaEnd():
    global g_va_buffer
    assert g_va_buffer != None
    objects_buffer.append(g_va_buffer)
    g_va_buffer = None

def vaLoadIdentity():
    global g_matrix_view, g_matrix_proj
    if g_matrix_mode == VA_MODELVIEW:
        g_matrix_view = np.asmatrix( np.identity(4, dtype=np.float) )
    else:
        g_matrix_proj = np.asmatrix( np.identity(4, dtype=np.float) )

def vaMatrixMode(matrix_mode):
    global g_matrix_mode
    g_matrix_mode = matrix_mode

def vaPushMatrix():
    """
    Push current matrix into stack and current matrix value remains the same.
    """
    global g_matrix_view, g_matrix_proj
    if g_matrix_mode == VA_MODELVIEW:
        view_mat_stack.append( g_matrix_view )
        g_matrix_view = np.copy( g_matrix_view )
    else:
        proj_mat_stack.append( g_matrix_proj )
        g_matrix_proj = np.copy( g_matrix_proj )

def vaPopMatrix():
    global g_matrix_view, g_matrix_proj
    try:
        if g_matrix_mode == VA_MODELVIEW:
            g_matrix_view = view_mat_stack.pop()
        else:
            g_matrix_proj = proj_mat_stack.pop()
    except:
        raise "Stack empty, not allowed to pop."

def vaPerspective(theta, aspect, dnear, dfar):
    global g_matrix_proj
    assert g_matrix_mode == VA_PROJECTION
    
    cotv = cos(theta / 2.0) / sin(theta / 2.0)
    m = np.asmatrix( np.zeros((4, 4), dtype=np.float) )
    m[0, 0] = cotv / aspect
    m[1, 1] = cotv
    m[2, 2] = float(- dnear - dfar) / (-dnear + dfar)
    m[3, 2] = -2.0 * (-dnear) * (-dfar) / (-dnear + dfar)
    m[2, 3] = -1.0

    g_matrix_proj = m * g_matrix_proj

def vaRotate(theta, x, y, z):
    global g_matrix_view
    assert g_matrix_mode == VA_MODELVIEW
    
    norm = sqrt(x * x + y * y + z * z)
    x, y, z = x / norm, y / norm, z / norm
    sinv = sin(theta)
    cosv = cos(theta)
    m = np.asmatrix( np.identity(4, dtype=np.float) )
    m[0, 0] = x * x * (1 - cosv) + cosv
    m[0, 1] = x * y * (1 - cosv) - z * sinv
    m[0, 2] = x * z * (1 - cosv) + y * sinv
    m[1, 0] = y * x * (1 - cosv) + z * sinv
    m[1, 1] = y * y * (1 - cosv) + cosv
    m[1, 2] = y * z * (1 - cosv) - x * sinv
    m[2, 0] = z * x * (1 - cosv) - y * sinv
    m[2, 1] = z * y * (1 - cosv) + x * sinv
    m[2, 2] = z * z * (1 - cosv) + cosv

    g_matrix_view = m * g_matrix_view 


def vaTranslate(x, y, z):
    global g_matrix_view
    assert g_matrix_mode == VA_MODELVIEW
    
    m = np.asmatrix( np.identity(4, dtype=np.float) )
    m[3, 0] = x
    m[3, 1] = y
    m[3, 2] = z

    g_matrix_view = m * g_matrix_view


def vaLookAt(cx, cy, cz, rx, ry, rz, ux, uy, uz):
    global g_matrix_view
    assert(g_matrix_mode == VA_MODELVIEW)
    
    c = np.array([cx, cy, cz], dtype=np.float)
    r = np.array([rx, ry, rz], dtype=np.float)
    u = np.array([ux, uy, uz], dtype=np.float)

    z = c - r
    z = z / np.linalg.norm(z)
    
    x = np.cross(u, z)
    x = x / np.linalg.norm(x)

    y = np.cross(z, x)
    y = y / np.linalg.norm(y)

    m = np.asmatrix( np.identity(4, dtype=np.float) )
    m[0, 0] = x[0]
    m[1, 0] = x[1]
    m[2, 0] = x[2]
    m[3, 0] = - c.dot(x)

    m[0, 1] = y[0]
    m[1, 1] = y[1]
    m[2, 1] = y[2]
    m[3, 1] = - c.dot(y)

    m[0, 2] = z[0]
    m[1, 2] = z[1]
    m[2, 2] = z[2]
    m[3, 2] = - c.dot(z)

    m[0, 3] = 0
    m[1, 3] = 0
    m[2, 3] = 0
    m[3, 3] = 1

    g_matrix_view = m * g_matrix_view

def vaClearBuffer():
    del objects_buffer[:]


""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
class ValleyGL(QtGui.QWidget):
    def __init__(self, parent=None):
        super(ValleyGL, self).__init__(parent)

        newFont = self.font()
        newFont.setPixelSize(12)
        self.setFont(newFont)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(event.rect(), QtGui.QBrush(QtCore.Qt.white))
        self.vaDisplayFunc()
        self.__drawObjects(painter)

    def __getWinCoord(self, x, y, z):
        w, h = self.width(), self.height()
        x, y = int((1 + x) * w / 2), int((1 + y) * h / 2)
        return x, y

    def __drawObjects(self, painter):
        for va in objects_buffer:
            if va.cmd == VA_POINTS:
                self.__drawPoints(painter, va)
            elif va.cmd == VA_LINES:
                self.__drawLines(painter, va)
            elif va.cmd == VA_LINE_LOOP:
                self.__drawLineLoop(painter, va)

    def __drawLineLoop(self, painter, va):
        transform = va.modelview * va.projection
        nr = len( va.pts_buffer )
        for i in xrange(nr):
            i_next = (i + 1) % nr
            p1 = va.pts_buffer[i] * transform
            p2 = va.pts_buffer[i_next] * transform
            p1 /= p1[0, 3]
            p2 /= p2[0, 3]
            x1, y1 = self.__getWinCoord(p1[0, 0], p1[0, 1], p1[0, 2])
            x2, y2 = self.__getWinCoord(p2[0, 0], p2[0, 1], p2[0, 2])
            painter.drawLine(x1, y1, x2, y2)
        
    def __drawPoints(self, painter, va):
        transform = va.modelview * va.projection
        for p in va.pts_buffer:
            p = p * transform
            p /= p[0, 3]    
            x, y = self.__getWinCoord(p[0, 0], p[0, 1], p[0, 2])
            painter.drawPoint(x, y)

    def __drawLines(self, painter, va):
        transform = va.modelview * va.projection
        for i in xrange(0, len(va.pts_buffer), 2):
            p1 = va.pts_buffer[i] * transform
            p2 = va.pts_buffer[i+1] * transform
            p1 /= p1[0, 3]
            p2 /= p2[0, 3]
            x1, y1 = self.__getWinCoord(p1[0, 0], p1[0, 1], p1[0, 2])
            x2, y2 = self.__getWinCoord(p2[0, 0], p2[0, 1], p2[0, 2])
            painter.drawLine(x1, y1, x2, y2)

    def vaDisplayFunc(self):
        pass
    


    




        
