"""
About Me:
website: http://www.ibaiyang.org
Author: Valley.He
Email: ubaiyang@gmail.com
"""

from valleygl import *

def drawCube():
    vaBegin(VA_LINE_LOOP)
    vaVertex(20, 20, 20)
    vaVertex(-20, 20, 20)
    vaVertex(-20, 20, -20)
    vaVertex(20, 20, -20)
    vaEnd()

    vaBegin(VA_LINE_LOOP)
    vaVertex(20, -20, 20)
    vaVertex(-20, -20, 20)
    vaVertex(-20, -20, -20)
    vaVertex(20, -20, -20)
    vaEnd()

    vaBegin(VA_LINES)
    vaVertex(20, -20, 20)
    vaVertex(20, 20, 20)
    
    vaVertex(-20, -20, -20)
    vaVertex(-20, 20, -20)
    
    vaVertex(-20, 20, 20)
    vaVertex(-20, -20, 20)
    
    vaVertex(20, -20, -20)
    vaVertex(20, 20, -20)
    vaEnd()

    
def displayFunc(w, h, thetaX, thetaY):
    vaClearBuffer()

    vaMatrixMode(VA_PROJECTION)
    vaLoadIdentity()
    vaPerspective(np.pi / 3, float(w) / h, 30, 1000)
    
    vaMatrixMode(VA_MODELVIEW)
    vaLoadIdentity()
    vaLookAt(40, 40, 50, 0, 0, 0, 0, 1, 0)
    
    vaRotate(np.deg2rad(thetaX), 0, 1, 0)
    vaRotate(np.deg2rad(thetaY), 1, 0, 0)
    drawCube()

    #store modelview matrix
    vaPushMatrix()
    vaTranslate(50, 0, 0)
    drawCube()
    
    vaPushMatrix()
    vaTranslate(0, 10,0)
    drawCube()
    vaPopMatrix()

    #restore previous matrix
    vaPopMatrix()

class Valley(ValleyGL):
    def __init__(self):
        super(Valley, self).__init__()
        self.lastPos = QtCore.QPoint()
        self.w, self.h = self.width(), self.height()
        self.thetaX, self.thetaY = 0, 0
        self.setWindowTitle("ValleyGL")

    def mouseMoveEvent(self, e):
        dx = e.pos().x() - self.lastPos.x()
        dy = e.pos().y() - self.lastPos.y()

        self.thetaX += dx
        self.thetaY += dy
        
        self.lastPos = e.pos()
        self.update()

    def mousePressEvent(self, e):
        self.lastPos = e.pos()
        self.update()

    def resizeEvent(self, e):
        self.w, self.h = self.width(), self.height()

    def vaDisplayFunc(self):
        displayFunc(self.w, self.h, self.thetaX, self.thetaY)

    def sizeHint(self):
        return QtCore.QSize(500, 300)

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    va = Valley()
    va.show()
    sys.exit(app.exec_())
